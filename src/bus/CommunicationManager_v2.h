#pragma once

// windows.h 会定义 ERROR 宏，与 EventSeverity 冲突
#ifdef ERROR
#undef ERROR
#endif

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common_types_extended.h"

namespace usv {

	// ============================================================================
	// 0. 通用消息基类
	// ============================================================================
	struct MessageHeader {
		uint64_t seq = 0;
		double timestamp = 0.0;
		std::string topic_name;
		std::string source_node;
		std::string frame_id;
	};

	struct RawDataMessage {
		MessageHeader header;
		virtual ~RawDataMessage() = default;
	};

	struct StateUpdateMessage {
		MessageHeader header;
		uint64_t state_version = 0;
		virtual ~StateUpdateMessage() = default;
	};

	enum class EventSeverity {
		Info = 0,
		Warning,
		ErrorLevel,
		Critical
	};

	struct CriticalEventMessage {
		MessageHeader header;
		std::string event_name;
		EventSeverity severity = EventSeverity::Info;
		virtual ~CriticalEventMessage() = default;
	};

	// ============================================================================
	// 1. Topic 基类
	// ============================================================================
	class IChannel {
	public:
		virtual ~IChannel() = default;
	};

	class ITopic : public IChannel {
	public:
		explicit ITopic(std::string name) : name_(std::move(name)) {}
		virtual ~ITopic() = default;

		const std::string& name() const { return name_; }

	private:
		std::string name_;
	};

	// ============================================================================
	// 2. 通用回调型 Topic（兼容原有 Channel 用法）
	// ============================================================================
	template <typename T>
	class Channel : public ITopic {
	public:
		using MessagePtr = std::shared_ptr<const T>;
		using Callback = std::function<void(MessagePtr)>;

		explicit Channel(const std::string& topic_name = "") : ITopic(topic_name) {}

		void subscribe(Callback cb) {
			std::lock_guard<std::mutex> lock(sub_mutex_);
			subscribers_.push_back(std::move(cb));
		}

		void publish(MessagePtr msg) {
			std::vector<Callback> callbacks_copy;
			{
				std::lock_guard<std::mutex> lock(sub_mutex_);
				callbacks_copy = subscribers_;
			}
			for (const auto& cb : callbacks_copy) {
				if (cb) {
					cb(msg);
				}
			}
		}

	private:
		std::vector<Callback> subscribers_;
		std::mutex sub_mutex_;
	};

	// ============================================================================
	// 3. 原始数据流 Topic: 有界环形缓存 + 多订阅者独立读指针
	// ============================================================================
	template <typename T>
	class RawTopic : public ITopic {
	public:
		using MessagePtr = std::shared_ptr<const T>;

		struct SubscriberCursor {
			uint64_t subscriber_id = 0;
			uint64_t next_seq = 0;
		};

		explicit RawTopic(const std::string& topic_name, size_t capacity = 32)
			: ITopic(topic_name), capacity_(capacity), buffer_(capacity) {}

		uint64_t addSubscriber() {
			std::lock_guard<std::mutex> lock(mutex_);
			uint64_t id = ++subscriber_id_gen_;
			subscribers_[id] = SubscriberCursor{ id, head_seq_ };
			return id;
		}

		void removeSubscriber(uint64_t subscriber_id) {
			std::lock_guard<std::mutex> lock(mutex_);
			subscribers_.erase(subscriber_id);
		}

		/** 将订阅者读指针跳到最新，避免中途启动时消费环形缓冲里的旧帧 */
		void seekSubscriberToLatest(uint64_t subscriber_id) {
			std::lock_guard<std::mutex> lock(mutex_);
			auto it = subscribers_.find(subscriber_id);
			if (it != subscribers_.end()) {
				it->second.next_seq = tail_seq_;
			}
		}

		void publish(MessagePtr msg) {
			std::lock_guard<std::mutex> lock(mutex_);
			const uint64_t seq = tail_seq_++;
			const size_t index = seq % capacity_;
			buffer_[index] = std::move(msg);

			if (tail_seq_ - head_seq_ > capacity_) {
				head_seq_ = tail_seq_ - capacity_;
			}
		}

		std::optional<MessagePtr> tryConsumeNext(uint64_t subscriber_id) {
			std::lock_guard<std::mutex> lock(mutex_);
			auto it = subscribers_.find(subscriber_id);
			if (it == subscribers_.end()) {
				return std::nullopt;
			}

			auto& cursor = it->second;
			if (cursor.next_seq < head_seq_) {
				cursor.next_seq = head_seq_;  //the slow user skip frame
			}
			if (cursor.next_seq >= tail_seq_) {
				return std::nullopt;
			}


			const size_t index = cursor.next_seq % capacity_;
			auto msg = buffer_[index];
			++cursor.next_seq;
			return msg;
		}

		uint64_t headSeq() const {
			std::lock_guard<std::mutex> lock(mutex_);
			return head_seq_;
		}

		uint64_t tailSeq() const {
			std::lock_guard<std::mutex> lock(mutex_);
			return tail_seq_;
		}

	private:
		size_t capacity_;
		std::vector<MessagePtr> buffer_;

		mutable std::mutex mutex_;
		uint64_t head_seq_ = 0;
		uint64_t tail_seq_ = 0;
		uint64_t subscriber_id_gen_ = 0;
		std::unordered_map<uint64_t, SubscriberCursor> subscribers_;
	};

	// ============================================================================
	// 4. 状态更新流 Topic: latest + short history + callback
	// ============================================================================
	template <typename T>
	class StateTopic : public ITopic {
	public:
		using MessagePtr = std::shared_ptr<const T>;
		using Callback = std::function<void(MessagePtr)>;

		explicit StateTopic(const std::string& topic_name, size_t history_size = 16)
			: ITopic(topic_name), history_size_(history_size) {}

		void publish(MessagePtr msg) {
			{
				std::unique_lock<std::shared_mutex> lock(data_mutex_);
				latest_ = msg;
				history_.push_back(msg);
				while (history_.size() > history_size_) {
					history_.pop_front();
				}
			}
			notify(msg);
		}

		std::shared_ptr<const T> getLatest() const {
			std::shared_lock<std::shared_mutex> lock(data_mutex_);
			return latest_;
		}

		std::vector<MessagePtr> getHistory() const {
			std::shared_lock<std::shared_mutex> lock(data_mutex_);
			return std::vector<MessagePtr>(history_.begin(), history_.end());
		}

		uint64_t subscribe(Callback cb) {
			std::unique_lock<std::shared_mutex> lock(cb_mutex_);
			const uint64_t id = ++callback_id_gen_;
			callbacks_[id] = std::move(cb);
			return id;
		}

		void unsubscribe(uint64_t id) {
			std::unique_lock<std::shared_mutex> lock(cb_mutex_);
			callbacks_.erase(id);
		}

	private:
		void notify(MessagePtr msg) {
			std::shared_lock<std::shared_mutex> lock(cb_mutex_);
			for (const auto& kv : callbacks_) {
				if (kv.second) {
					kv.second(msg);
				}
			}
		}

	private:
		size_t history_size_;
		std::shared_ptr<const T> latest_;
		std::deque<MessagePtr> history_;

		mutable std::shared_mutex data_mutex_;
		mutable std::shared_mutex cb_mutex_;
		uint64_t callback_id_gen_ = 0;
		std::unordered_map<uint64_t, Callback> callbacks_;
	};

	// ============================================================================
	// 5. 关键事件流 Topic: 事件保留 + callback
	// ============================================================================
	template <typename T>
	class EventTopic : public ITopic {
	public:
		using MessagePtr = std::shared_ptr<const T>;
		using Callback = std::function<void(MessagePtr)>;

		explicit EventTopic(const std::string& topic_name, size_t retain_size = 128)
			: ITopic(topic_name), retain_size_(retain_size) {}

		void publish(MessagePtr msg) {
			{
				std::unique_lock<std::shared_mutex> lock(data_mutex_);
				retained_.push_back(msg);
				while (retained_.size() > retain_size_) {
					retained_.pop_front();
				}
			}
			notify(msg);
		}

		std::vector<MessagePtr> getRetained() const {
			std::shared_lock<std::shared_mutex> lock(data_mutex_);
			return std::vector<MessagePtr>(retained_.begin(), retained_.end());
		}

		uint64_t subscribe(Callback cb) {
			std::unique_lock<std::shared_mutex> lock(cb_mutex_);
			const uint64_t id = ++callback_id_gen_;
			callbacks_[id] = std::move(cb);
			return id;
		}

		void unsubscribe(uint64_t id) {
			std::unique_lock<std::shared_mutex> lock(cb_mutex_);
			callbacks_.erase(id);
		}

	private:
		void notify(MessagePtr msg) {
			std::shared_lock<std::shared_mutex> lock(cb_mutex_);
			for (const auto& kv : callbacks_) {
				if (kv.second) {
					kv.second(msg);
				}
			}
		}

	private:
		size_t retain_size_;
		std::deque<MessagePtr> retained_;

		mutable std::shared_mutex data_mutex_;
		mutable std::shared_mutex cb_mutex_;
		uint64_t callback_id_gen_ = 0;
		std::unordered_map<uint64_t, Callback> callbacks_;
	};

	// ============================================================================
	// 6. 状态仓库：保存“当前最新状态”
	// ============================================================================
	class StateRepository {
	public:
		template <typename T>
		void update(std::shared_ptr<const T> state) {
			std::unique_lock<std::shared_mutex> lock(mutex_);
			states_[std::type_index(typeid(T))] = state;
		}

		template <typename T>
		std::shared_ptr<const T> getLatest() const {
			std::shared_lock<std::shared_mutex> lock(mutex_);
			auto it = states_.find(std::type_index(typeid(T)));
			if (it == states_.end()) {
				return nullptr;
			}
			return std::static_pointer_cast<const T>(it->second);
		}

	private:
		mutable std::shared_mutex mutex_;
		std::unordered_map<std::type_index, std::shared_ptr<const void>> states_;
	};

	// ============================================================================
	// 7. 地图仓库：KeyFrame + Block + Snapshot + 版本管理
	// ============================================================================
	class MapRepository {
	public:
		uint64_t addOrUpdateBlock(std::shared_ptr<MapBlock> block) {
			std::unique_lock<std::shared_mutex> lock(mutex_);
			const uint64_t version = ++version_;
			block->version = version;
			block->dirty = true;
			blocks_[block->key] = std::move(block);
			return version;
		}

		uint64_t addKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
			std::unique_lock<std::shared_mutex> lock(mutex_);
			const uint64_t version = ++version_;
			keyframes_[keyframe->id] = std::move(keyframe);
			return version;
		}

		std::shared_ptr<const MapSnapshot> getSnapshot() const {
			std::shared_lock<std::shared_mutex> lock(mutex_);
			auto snapshot = std::make_shared<MapSnapshot>();
			snapshot->version = version_.load();
			snapshot->timestamp = 0.0;
			for (const auto& kv : blocks_) {
				snapshot->blocks.push_back(kv.second);
			}
			for (const auto& kv : keyframes_) {
				snapshot->keyframe_poses.push_back(kv.second->pose_wb);
			}
			return snapshot;
		}

		std::vector<std::shared_ptr<const MapBlock>> queryBlocks(const BoundingBox3D& region) const {
			std::shared_lock<std::shared_mutex> lock(mutex_);
			std::vector<std::shared_ptr<const MapBlock>> result;
			for (const auto& kv : blocks_) {
				if (intersect(kv.second->bbox, region)) {
					result.push_back(kv.second);
				}
			}
			return result;
		}

		std::vector<std::shared_ptr<const MapBlock>> getDirtyBlocksSince(uint64_t base_version) const {
			std::shared_lock<std::shared_mutex> lock(mutex_);
			std::vector<std::shared_ptr<const MapBlock>> result;
			for (const auto& kv : blocks_) {
				if (kv.second->version > base_version) {
					result.push_back(kv.second);
				}
			}
			return result;
		}

		uint64_t currentVersion() const {
			return version_.load();
		}

	private:
		static bool intersect(const BoundingBox3D& a, const BoundingBox3D& b) {
			return !(a.max_x < b.min_x || a.min_x > b.max_x ||
				a.max_y < b.min_y || a.min_y > b.max_y ||
				a.max_z < b.min_z || a.min_z > b.max_z);
		}

	private:
		mutable std::shared_mutex mutex_;
		std::unordered_map<BlockKey, std::shared_ptr<MapBlock>, BlockKeyHasher> blocks_;
		std::unordered_map<uint64_t, std::shared_ptr<KeyFrame>> keyframes_;
		std::atomic<uint64_t> version_{ 0 };
	};

	// ============================================================================
	// 8. 事件日志仓库
	// ============================================================================
	class EventLogRepository {
	public:
		template <typename T>
		void append(std::shared_ptr<const T> event) {
			std::unique_lock<std::shared_mutex> lock(mutex_);
			events_.push_back(event);
			while (events_.size() > max_size_) {
				events_.pop_front();
			}
		}

		std::vector<std::shared_ptr<const void>> getAll() const {
			std::shared_lock<std::shared_mutex> lock(mutex_);
			return std::vector<std::shared_ptr<const void>>(events_.begin(), events_.end());
		}

	private:
		size_t max_size_ = 1024;
		mutable std::shared_mutex mutex_;
		std::deque<std::shared_ptr<const void>> events_;
	};

	// ============================================================================
	// 9. 通信管理器：统一入口
	// ============================================================================
	class CommunicationManager {
	public:
		static CommunicationManager& instance() {
			static CommunicationManager inst;
			return inst;
		}

		template <typename T>
		std::shared_ptr<Channel<T>> getChannel(const std::string& topic_name) {
			{
				std::shared_lock<std::shared_mutex> read_lock(map_mutex_);
				auto it = topics_.find(topic_name);
				if (it != topics_.end()) {
					return std::dynamic_pointer_cast<Channel<T>>(it->second);
				}
			}
			{
				std::unique_lock<std::shared_mutex> write_lock(map_mutex_);
				auto it = topics_.find(topic_name);
				if (it != topics_.end()) {
					return std::dynamic_pointer_cast<Channel<T>>(it->second);
				}
				auto new_topic = std::make_shared<Channel<T>>(topic_name);
				topics_[topic_name] = new_topic;
				return new_topic;
			}
		}

		template <typename T>
		std::shared_ptr<RawTopic<T>> getRawTopic(const std::string& topic_name, size_t capacity = 32) {
			return getOrCreateTopic<RawTopic<T>>(topic_name, capacity);
		}

		template <typename T>
		std::shared_ptr<StateTopic<T>> getStateTopic(const std::string& topic_name, size_t history_size = 16) {
			return getOrCreateTopic<StateTopic<T>>(topic_name, history_size);
		}

		template <typename T>
		std::shared_ptr<EventTopic<T>> getEventTopic(const std::string& topic_name, size_t retain_size = 128) {
			return getOrCreateTopic<EventTopic<T>>(topic_name, retain_size);
		}

		template <typename T>
		void updateStateRepository(std::shared_ptr<const T> state) {
			state_repo_.update<T>(std::move(state));
		}

		template <typename T>
		std::shared_ptr<const T> getLatestState() const {
			return state_repo_.getLatest<T>();
		}

		MapRepository& mapRepository() { return map_repo_; }
		const MapRepository& mapRepository() const { return map_repo_; }

		EventLogRepository& eventLogRepository() { return event_log_repo_; }
		const EventLogRepository& eventLogRepository() const { return event_log_repo_; }

		void start() {
			std::cout << "[CommManager] Started locally with raw/state/event topics and repositories." << std::endl;
		}

		void stop() {
			std::unique_lock<std::shared_mutex> write_lock(map_mutex_);
			topics_.clear();
			std::cout << "[CommManager] Stopped and cleared all topics." << std::endl;
		}

	private:
		CommunicationManager() = default;
		~CommunicationManager() = default;
		CommunicationManager(const CommunicationManager&) = delete;
		CommunicationManager& operator=(const CommunicationManager&) = delete;

		template <typename TopicT, typename... Args>
		std::shared_ptr<TopicT> getOrCreateTopic(const std::string& topic_name, Args&&... args) {
			{
				std::shared_lock<std::shared_mutex> read_lock(map_mutex_);
				auto it = topics_.find(topic_name);
				if (it != topics_.end()) {
					return std::dynamic_pointer_cast<TopicT>(it->second);
				}
			}
			{
				std::unique_lock<std::shared_mutex> write_lock(map_mutex_);
				auto it = topics_.find(topic_name);
				if (it != topics_.end()) {
					return std::dynamic_pointer_cast<TopicT>(it->second);
				}
				auto new_topic = std::make_shared<TopicT>(topic_name, std::forward<Args>(args)...);
				topics_[topic_name] = new_topic;
				return new_topic;
			}
		}

	private:
		std::unordered_map<std::string, std::shared_ptr<ITopic>> topics_;
		mutable std::shared_mutex map_mutex_;

		StateRepository state_repo_;
		MapRepository map_repo_;
		EventLogRepository event_log_repo_;
	};

} // namespace usv
