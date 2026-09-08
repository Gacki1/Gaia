#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <vector>
#include <functional>
#include <cstddef>

namespace planet {

class JobSystem {
public:
    JobSystem() = default;
    ~JobSystem() { stop(); }

    JobSystem(const JobSystem&)            = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    void start(unsigned n = 0) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!workers_.empty()) return;
        if (n == 0) {
            const unsigned hw = std::thread::hardware_concurrency();
            n = hw > 1 ? hw - 1 : 1;
        }
        stopping_ = false;
        workers_.reserve(n);
        for (unsigned i = 0; i < n; ++i)
            workers_.emplace_back([this] { workerLoop(); });
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (workers_.empty()) return;
            stopping_ = true;
            queue_.clear();
        }
        cv_.notify_all();
        for (std::thread& t : workers_) if (t.joinable()) t.join();
        workers_.clear();
        std::lock_guard<std::mutex> lock(mtx_);
        running_ = 0;
    }

    void enqueue(std::function<void()> job) {
        bool queued = false;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!workers_.empty()) {
                queue_.push_back(std::move(job));
                queued = true;
            }
        }
        if (queued) cv_.notify_one();
        else        job();
    }

    std::size_t pending() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size() + running_;
    }

    bool threaded() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return !workers_.empty();
    }

private:
    void workerLoop() {
        for (;;) {
            std::function<void()> job;
            {
                std::unique_lock<std::mutex> lock(mtx_);
                cv_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
                if (stopping_) return;
                job = std::move(queue_.front());
                queue_.pop_front();
                ++running_;
            }
            job();
            {
                std::lock_guard<std::mutex> lock(mtx_);
                --running_;
            }
        }
    }

    mutable std::mutex                mtx_;
    std::condition_variable           cv_;
    std::deque<std::function<void()>> queue_;
    std::vector<std::thread>          workers_;
    std::size_t                       running_  = 0;
    bool                              stopping_ = false;
};

}
