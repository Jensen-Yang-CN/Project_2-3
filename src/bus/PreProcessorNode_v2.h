#pragma once

#include "CommunicationManager_v2.h"
#include "SystemFileLogger.h"
#include "common_types_extended.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace usv {

/**
 * PreProcessorNodeV2：订阅 210/211 点云与左相机 RawTopic，以 211 为主时钟生成同步包。
 * IMU/GNSS 已独立发布到 /sensor/imu/raw，本节点当前不强制绑定。
 */
class PreProcessorNodeV2 {
public:
    struct Config {
        std::string lidar210_topic = "/sensor/lidar/210/raw";
        std::string lidar211_topic = "/sensor/lidar/211/raw";
        std::string image_topic = "/sensor/camera/left/raw";
        std::string bundle_topic = "/preprocess/synced_bundle";

        size_t raw_topic_capacity = 64;
        size_t output_history_size = 16;
        size_t max_image_buffer_size = 32;
        size_t max_lidar210_buffer_size = 16;

        double poll_sleep_ms = 2.0;
        double max_lidar_image_time_diff_sec = 0.08;
        double max_lidar210_211_time_diff_sec = 0.15;
        double max_stale_image_fallback_sec = 0.5;
        bool use_latest_image_fallback = true;

        bool require_image = false;
        bool require_lidar210 = false;
    };

    PreProcessorNodeV2()
        : PreProcessorNodeV2(Config{})
    {
    }

    explicit PreProcessorNodeV2(const Config &cfg)
        : cfg_(cfg)
        , comm_(&CommunicationManager::instance())
    {
    }

    ~PreProcessorNodeV2() { Stop(); }

    void Init()
    {
        lidar210_topic_ = comm_->getRawTopic<LidarFrame>(cfg_.lidar210_topic, cfg_.raw_topic_capacity);
        lidar211_topic_ = comm_->getRawTopic<LidarFrame>(cfg_.lidar211_topic, cfg_.raw_topic_capacity);
        image_topic_ = comm_->getRawTopic<ImageFrame>(cfg_.image_topic, cfg_.raw_topic_capacity);
        bundle_topic_ = comm_->getStateTopic<PreprocessedSensorBundle>(
            cfg_.bundle_topic, cfg_.output_history_size);

        lidar210_sub_id_ = lidar210_topic_->addSubscriber();
        lidar211_sub_id_ = lidar211_topic_->addSubscriber();
        image_sub_id_ = image_topic_->addSubscriber();
    }

    void Start()
    {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true))
            return;
        worker_ = std::thread(&PreProcessorNodeV2::ProcessLoop, this);
        SystemFileLogger::instance().logLifecycle("PreProcessor", "start");
    }

    void Stop()
    {
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false))
            return;
        if (worker_.joinable())
            worker_.join();
        SystemFileLogger::instance().logLifecycle("PreProcessor", "stop");
    }

    void requestPlaybackTimelineReset(uint64_t timelineGeneration = 0)
    {
        const uint64_t nextGeneration = timelineGeneration != 0
            ? timelineGeneration
            : playback_timeline_generation_.load(std::memory_order_acquire) + 1;
        playback_timeline_generation_.store(nextGeneration, std::memory_order_release);
        playback_timeline_reset_requested_.store(true, std::memory_order_release);
    }

private:
    struct ImageMatch {
        std::shared_ptr<const ImageFrame> image;
        double time_diff_sec = std::numeric_limits<double>::max();
    };

    struct Lidar210Match {
        std::shared_ptr<const LidarFrame> frame;
        double time_diff_sec = std::numeric_limits<double>::max();
    };

    void ProcessLoop()
    {
        while (running_) {
            if (playback_timeline_reset_requested_.exchange(
                    false, std::memory_order_acq_rel)) {
                resetPlaybackTimeline();
            }
            DrainImageTopic();
            DrainLidar210Topic();

            bool processed = false;
            while (auto maybe = lidar211_topic_->tryConsumeNext(lidar211_sub_id_)) {
                processed = true;
                OnLidar211Received(*maybe);
                DrainImageTopic();
                DrainLidar210Topic();
            }

            if (!processed) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(cfg_.poll_sleep_ms)));
            }
        }
    }

    void resetPlaybackTimeline()
    {
        while (lidar210_topic_->tryConsumeNext(lidar210_sub_id_)) {}
        while (lidar211_topic_->tryConsumeNext(lidar211_sub_id_)) {}
        while (image_topic_->tryConsumeNext(image_sub_id_)) {}
        std::lock_guard<std::mutex> lock(data_mutex_);
        image_buffer_.clear();
        lidar210_buffer_.clear();
    }

    void DrainImageTopic()
    {
        while (auto maybe = image_topic_->tryConsumeNext(image_sub_id_)) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            image_buffer_.push_back(*maybe);
            while (image_buffer_.size() > cfg_.max_image_buffer_size)
                image_buffer_.pop_front();
        }
    }

    void DrainLidar210Topic()
    {
        while (auto maybe = lidar210_topic_->tryConsumeNext(lidar210_sub_id_)) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            lidar210_buffer_.push_back(*maybe);
            while (lidar210_buffer_.size() > cfg_.max_lidar210_buffer_size)
                lidar210_buffer_.pop_front();
        }
    }

    void OnLidar211Received(const std::shared_ptr<const LidarFrame> &raw211)
    {
        const uint64_t timelineGeneration =
            playback_timeline_generation_.load(std::memory_order_acquire);
        if (!raw211 || raw211->timeline_generation != timelineGeneration)
            return;
        const double t = raw211->timestamp;
        auto bundle = std::make_shared<PreprocessedSensorBundle>();
        bundle->timestamp = t;
        bundle->timeline_generation = timelineGeneration;
        bundle->raw_lidar_211 = raw211;
        bundle->deskewed_lidar_211 = raw211;

        bool drop = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            auto img_match = FindClosestImage(t);
            if (img_match.has_value()
                && img_match->image->timeline_generation == timelineGeneration
                && img_match->time_diff_sec <= cfg_.max_lidar_image_time_diff_sec) {
                bundle->synced_image = img_match->image;
                bundle->image_found = true;
            } else if (cfg_.use_latest_image_fallback && !image_buffer_.empty()) {
                const auto &latest = image_buffer_.back();
                if (latest
                    && latest->timeline_generation == timelineGeneration
                    && std::abs(latest->timestamp - t) <= cfg_.max_stale_image_fallback_sec) {
                    bundle->synced_image = latest;
                    bundle->image_found = true;
                }
            }
            if (!bundle->image_found && cfg_.require_image) {
                drop = true;
            }

            auto l210_match = FindClosestLidar210(t);
            if (l210_match.has_value()
                && l210_match->frame->timeline_generation == timelineGeneration
                && l210_match->time_diff_sec <= cfg_.max_lidar210_211_time_diff_sec) {
                bundle->raw_lidar_210 = l210_match->frame;
                bundle->deskewed_lidar_210 = l210_match->frame;
                bundle->lidar_210_found = true;
            } else if (cfg_.require_lidar210) {
                drop = true;
            }
        }

        if (drop)
            return;

        if (timelineGeneration
            != playback_timeline_generation_.load(std::memory_order_acquire)) {
            return;
        }

        bundle_topic_->publish(bundle);
        comm_->updateStateRepository<PreprocessedSensorBundle>(bundle);
    }

    std::optional<ImageMatch> FindClosestImage(double timestamp) const
    {
        if (image_buffer_.empty())
            return std::nullopt;
        auto it = std::min_element(
            image_buffer_.begin(), image_buffer_.end(),
            [timestamp](const std::shared_ptr<const ImageFrame> &a,
                        const std::shared_ptr<const ImageFrame> &b) {
                return std::abs(a->timestamp - timestamp) < std::abs(b->timestamp - timestamp);
            });
        if (it == image_buffer_.end() || !*it)
            return std::nullopt;
        ImageMatch m;
        m.image = *it;
        m.time_diff_sec = std::abs((*it)->timestamp - timestamp);
        return m;
    }

    std::optional<Lidar210Match> FindClosestLidar210(double timestamp) const
    {
        if (lidar210_buffer_.empty())
            return std::nullopt;
        auto it = std::min_element(
            lidar210_buffer_.begin(), lidar210_buffer_.end(),
            [timestamp](const std::shared_ptr<const LidarFrame> &a,
                        const std::shared_ptr<const LidarFrame> &b) {
                return std::abs(a->timestamp - timestamp) < std::abs(b->timestamp - timestamp);
            });
        if (it == lidar210_buffer_.end() || !*it)
            return std::nullopt;
        Lidar210Match m;
        m.frame = *it;
        m.time_diff_sec = std::abs((*it)->timestamp - timestamp);
        return m;
    }

    Config cfg_;
    CommunicationManager *comm_ = nullptr;

    std::shared_ptr<RawTopic<LidarFrame>> lidar210_topic_;
    std::shared_ptr<RawTopic<LidarFrame>> lidar211_topic_;
    std::shared_ptr<RawTopic<ImageFrame>> image_topic_;
    std::shared_ptr<StateTopic<PreprocessedSensorBundle>> bundle_topic_;

    uint64_t lidar210_sub_id_ = 0;
    uint64_t lidar211_sub_id_ = 0;
    uint64_t image_sub_id_ = 0;

    mutable std::mutex data_mutex_;
    std::deque<std::shared_ptr<const ImageFrame>> image_buffer_;
    std::deque<std::shared_ptr<const LidarFrame>> lidar210_buffer_;

    std::atomic<bool> running_{false};
    std::atomic<bool> playback_timeline_reset_requested_{false};
    std::atomic<uint64_t> playback_timeline_generation_{1};
    std::thread worker_;
};

} // namespace usv
