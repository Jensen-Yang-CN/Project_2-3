#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

namespace usv {

template <typename T>
class ThreadSafeQueue {
public:
    explicit ThreadSafeQueue(size_t max_capacity = 0)
        : max_capacity_(max_capacity)
    {
    }

    ~ThreadSafeQueue() { shutdown(); }

    ThreadSafeQueue(const ThreadSafeQueue &) = delete;
    ThreadSafeQueue &operator=(const ThreadSafeQueue &) = delete;

    void push(T value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_shutdown_)
            return;
        if (max_capacity_ > 0 && queue_.size() >= max_capacity_) {
            queue_.pop();
            dropped_count_++;
        }
        queue_.push(std::move(value));
        cond_var_.notify_one();
    }

    bool pop(T &value)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_var_.wait(lock, [this] { return !queue_.empty() || is_shutdown_; });
        if (queue_.empty() && is_shutdown_)
            return false;
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    bool try_pop(T &value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty())
            return false;
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty())
            queue_.pop();
    }

    void shutdown()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            is_shutdown_ = true;
        }
        cond_var_.notify_all();
    }

    /** ֹͣ���ٴ�����ǰ���ã���� shutdown ����ն��� */
    void reopen()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        is_shutdown_ = false;
        while (!queue_.empty())
            queue_.pop();
    }

    /** 返回因达到容量上限而被丢弃的元素数，用于运行期诊断。 */
    size_t droppedCount() const
    {
        return dropped_count_.load(std::memory_order_relaxed);
    }

private:
    std::queue<T> queue_;
    size_t max_capacity_ = 0;
    mutable std::mutex mutex_;
    std::condition_variable cond_var_;
    bool is_shutdown_ = false;
    std::atomic<size_t> dropped_count_{0};
};

} // namespace usv
