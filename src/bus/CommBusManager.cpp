#include "CommBusManager.h"

#include "SystemFileLogger.h"

#include <QObject>

CommBusManager &CommBusManager::instance()
{
    static CommBusManager inst;
    return inst;
}

void CommBusManager::ensurePublisher()
{
    if (publisher_)
        return;
    usv::CommunicationManager::instance().start();
    publisher_ = std::make_unique<SensorPublisherNode>();
    publisher_->Init();
    SystemFileLogger::instance().logLifecycle("SensorPublisher", "ready");
}

SensorPublisherNode *CommBusManager::sensorPublisher()
{
    return publisher_.get();
}

void CommBusManager::ensureInitialized()
{
    ensurePublisher();
    if (core_inited_)
        return;

    usv::PreProcessorNodeV2::Config pre_cfg;
    pre_cfg.lidar210_topic = "/sensor/lidar/210/raw";
    pre_cfg.lidar211_topic = "/sensor/lidar/211/raw";
    pre_cfg.image_topic = "/sensor/camera/left/raw";
    pre_cfg.bundle_topic = "/preprocess/synced_bundle";
    pre_cfg.require_image = false;
    pre_cfg.require_lidar210 = false;
    pre_cfg.max_lidar_image_time_diff_sec = 0.20;
    pre_cfg.max_lidar210_211_time_diff_sec = 0.15;
    pre_cfg.max_stale_image_fallback_sec = 0.50;
    pre_cfg.use_latest_image_fallback = true;

    preprocessor_ = std::make_unique<usv::PreProcessorNodeV2>(pre_cfg);
    preprocessor_->Init();
    SystemFileLogger::instance().logLifecycle("PreProcessor", "ready");

    core_inited_ = true;
}

void CommBusManager::ensureBridgeNode()
{
    ensureInitialized();
    if (bridge_node_)
        return;
    bridge_node_ = std::make_unique<BridgeDetectionNode>();
    bridge_node_->Init();
}

void CommBusManager::ensureBerthNode()
{
    ensureInitialized();
    if (berth_node_)
        return;
    berth_node_ = std::make_unique<BerthDetectionNode>();
    berth_node_->Init();
}

void CommBusManager::startPipeline()
{
    ensureBridgeNode();
    ensureBerthNode();
    if (pipeline_running_)
        return;
    SystemFileLogger::instance().logLifecycle("PerceptionPipeline", "start");
    preprocessor_->Start();
    bridge_node_->Start();
    berth_node_->Start();
    pipeline_running_ = true;
}

void CommBusManager::stopPipeline()
{
    if (!pipeline_running_)
        return;
    SystemFileLogger::instance().logLifecycle("PerceptionPipeline", "stop");
    if (bridge_node_)
        bridge_node_->stop();
    if (berth_node_)
        berth_node_->stop();
    if (preprocessor_)
        preprocessor_->Stop();
    pipeline_running_ = false;
}

void CommBusManager::requestPlaybackTimelineReset(uint64_t sensorTimelineGeneration)
{
    if (preprocessor_)
        preprocessor_->requestPlaybackTimelineReset(sensorTimelineGeneration);
    if (bridge_node_)
        bridge_node_->requestPlaybackTimelineReset();
    if (berth_node_)
        berth_node_->requestPlaybackTimelineReset();
}
