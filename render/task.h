#ifndef RENDER_DEBUG_TASK_H
#define RENDER_DEBUG_TASK_H

#include <mutex>
#include <deque>
#include <vector>
#include <thread>
#include <chrono>
#include <memory>
#include <condition_variable>

#include <fmt/format.h>


// fixme 条件变量的写法容易出错，留意一下
// 这个类并不关心临界区，只关心等待和通知唤醒
class Event {
public:
    Event() = default;

    // 阻塞等待，直到被唤醒或者超时
    inline void wait_for(int ms) {
        assert(ms > 0);
        std::unique_lock<std::mutex> lck(_mtx);
        _cdt.wait_for(lck, std::chrono::milliseconds(ms));
    }

    // 通知等待的线程
    inline void notify() { _cdt.notify_one(); }

private:
    std::mutex _mtx{};
    std::condition_variable _cdt{};
};


// 🐝：用于做计算的执行者
template<class Task, class Result>
class Bee {
public:
    typedef std::function<Result(const Task &)> JobType;

    Bee(const JobType &job, Event &event)
            : _job(job),
              _event(event),
              _task_buffer(), _result_buffer(),
              _is_task_complete(true) {}


    // main: 检查这个 worker 是否将分配的任务完成了
    [[nodiscard]] inline bool is_task_complete() const { return _is_task_complete; }

    // 取出结果
    inline std::vector<Result> result_get() {
        assert(_is_task_complete);

        auto res = _result_buffer;
        _result_buffer.clear();
        return res;
    }

    // main: 分配任务，让线程执行
    void assign_tasks(const std::vector<Task> &tasks) {
        // 参数校验：不为空
        assert(!tasks.empty());

        // 内部状态校验：再分配任务之前，内部任务一定是已经完成了的
        assert(_task_buffer.empty());
        assert(_is_task_complete);

        _is_task_complete = false;
        _task_buffer = tasks;
        std::thread t(&Bee::_thread_run, this);
        t.detach();
    }


private:
    // worker: 线程实际执行的内容
    void _thread_run() {
        // 内部状态校验：执行任务前，任务列表一定不为空，结果列表一定为空
        assert(!_task_buffer.empty());
        assert(_result_buffer.empty());

        while (!_task_buffer.empty()) {
            // 取出任务
            Task task = _task_buffer.back();
            _task_buffer.pop_back();

            // 执行任务，产生结果
            Result result = _job(task);
            _result_buffer.push_back(result);
        }

        // 通知管理者
        _is_task_complete = true;
        _event.notify();
    }


private:
    JobType _job;
    Event &_event;

    std::vector<Task> _task_buffer;
    std::vector<Result> _result_buffer;

    bool _is_task_complete;
};


// 🐜：数据处理
template<class Result>
class Ant {
public:
    typedef std::function<void(const Result &)> JobType;

    // 创建负责数据处理的 worker，创建后就开始执行了
    Ant(const JobType &job, unsigned result_buffer_size, unsigned total_result_cnt)
            : _job(job),
              _mtx(),
              _result_buffer_size(result_buffer_size),
              _total_result_cnt(total_result_cnt),
              _result_list(), _total_solved(0) {

        // 让线程开始运行
        std::thread t(&Ant::_thread_run, this);
        t.detach();
    }

    // main：查看任务列表堆积了多少个任务
    [[nodiscard]] inline unsigned int result_list_cnt() const { return this->_result_list.size(); }

    // main: 查看总共处理了多少个 result
    [[nodiscard]] inline unsigned int total_solved() const { return _total_solved; }

    // main: 放入结果
    inline void push_result(const std::vector<Result> &res) {
        assert(!res.empty());
        assert(_total_solved < _total_result_cnt);

        std::lock_guard<std::mutex> lck(_mtx);
        _result_list.insert(_result_list.end(), res.begin(), res.end());
    }


private:
    // worker: 线程执行的内容
    void _thread_run() {
        while (_total_solved < _total_result_cnt) {
            // 如果列表里什么的没有，就等等
            if (_result_list.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(_sleep_interval_ms));
                continue;
            }

            // 开始处理数据
            auto result_buffer = _pop_result(_result_buffer_size);
            for (const auto &result : result_buffer) {
                _job(result);
                ++_total_solved;
            }
        }
    }

    // worker: 从结果缓存中取出几条，可能一条也取不到
    std::vector<Result> _pop_result(unsigned int res_cnt) {
        // 参数：合法
        assert(res_cnt > 0);

        std::lock_guard<std::mutex> lck(_mtx);
        std::vector<Result> res;
        for (unsigned int i = 0; i < res_cnt && !_result_list.empty(); ++i) {
            res.push_back(_result_list.back());
            _result_list.pop_back();
        }
        return res;
    }

private:
    JobType _job;
    std::mutex _mtx;        // _mtx 用于保护 _result_list

    unsigned int _result_buffer_size;
    unsigned int _total_result_cnt;

    std::deque<Result> _result_list;        // 存放总任务
    unsigned int _total_solved;

    // todo 这个值可以作为参数
    const unsigned int _sleep_interval_ms = 50;
};


// 🦒：任务的管理者
template<class Task, class Result>
class Giraffe {
public:
    Giraffe(unsigned bee_cnt, unsigned int bee_buffer, unsigned ant_buffer,
            unsigned wait_internal_ms,
            const typename Bee<Task, Result>::JobType &bee_job,
            const typename Ant<Result>::JobType &ant_job,
            const std::deque<Task> &task_list)
            : _bee_cnt(bee_cnt), _bee_buffer(bee_buffer), _ant_buffer(ant_buffer),
              _wait_internal_ms(wait_internal_ms),
              _workers(), _ant(),
              _event(),
              _total_task_cnt(task_list.size()), _task_queue(task_list) {
        assert(wait_internal_ms > 0);
        assert(bee_cnt > 0);
        assert(bee_buffer > 0);
        assert(ant_buffer > 0);
        assert(!task_list.empty());

        // 创建 bee
        for (unsigned int i = 0; i < _bee_cnt; ++i) {
            _workers.push_back(std::make_shared<Bee<Task, Result>>(bee_job, _event));
        }

        // 创建 ant
        _ant = std::make_shared<Ant<Result>>(ant_job, _ant_buffer, _total_task_cnt);
    }

    // main: 运行起来
    void run() {
        auto start_time = std::chrono::system_clock::now();
        while (_ant->total_solved() < _total_task_cnt) {
            std::vector<Result> result_buffer;

            for (auto &bee : _workers) {
                if (!bee->is_task_complete())
                    continue;

                // 收集结果
                auto bee_results = bee->result_get();
                result_buffer.insert(result_buffer.end(), bee_results.begin(), bee_results.end());

                // 重新分配任务
                if (_task_queue.empty())
                    continue;
                std::vector<Task> tasks;
                for (int i = 0; i < _bee_buffer && !_task_queue.empty(); ++i) {
                    tasks.push_back(_task_queue.front());
                    _task_queue.pop_front();
                }
                bee->assign_tasks(tasks);
            }

            // 让 ant 去处理结果
            if (!result_buffer.empty()) {
                _ant->push_result(result_buffer);
            }

            // 更新进度，统计速度
            auto current_time = std::chrono::system_clock::now();
            auto time_passed_s = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
            auto speed_task = (_total_task_cnt - _task_queue.size()) / time_passed_s.count();
            auto speed_result = _ant->total_solved() / time_passed_s.count();
            fmt::print("\r tasks alloc: {}/{}, result process: {}/{}",
                       _total_task_cnt - _task_queue.size(), _total_task_cnt, _ant->total_solved(), _total_task_cnt);
            fmt::print(" ,task speed: {}, result speed: {}", speed_task, speed_result);
            fflush(stdout);

            _event.wait_for(_wait_internal_ms);
        }
    }

private:
    const unsigned int _bee_cnt;                // worker 的数量
    const unsigned int _bee_buffer;             // main 分配 task 的速度
    const unsigned int _ant_buffer;             // ant 处理 result 的速度
    const unsigned int _wait_internal_ms;       // 主线程调度等待的间隔

    std::vector<std::shared_ptr<Bee<Task, Result>>> _workers;
    std::shared_ptr<Ant<Result>> _ant;
    Event _event;

    unsigned int _total_task_cnt;
    std::deque<Task> _task_queue;
};


#endif //RENDER_DEBUG_TASK_H
