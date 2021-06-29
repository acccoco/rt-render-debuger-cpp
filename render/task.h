#ifndef RENDER_DEBUG_TASK_H
#define RENDER_DEBUG_TASK_H

#include <mutex>
#include <deque>
#include <vector>
#include <thread>
#include <chrono>
#include <memory>

#include <fmt/format.h>
#include <spdlog/spdlog.h>


/**
 * 执行任务的 worker，不断地从任务列表中取出任务，将结果写入结果列表中
 * @tparam TaskT__ 任务的类型
 * @tparam ResultT__ 结果的类型
 * @使用方法
 *  Worker worker();
 *  worker.start();
 *  worker.stop();
 */
template<class TaskT_, class ResultT_>
class Worker {
public:
    typedef std::function<ResultT_(const TaskT_ &)> JobType;

private:
    const size_t BUFFER_SIZE;                       /* task buffer 和 result buffer 的容量是多大 */
    const unsigned WAIT_SLEEP_MS;                   /* 获取不到任务时，sleep 多久 */

public:
    /**
     * 初始化 workder
     * @param task_list 读取任务的列表
     * @param result_list 写入最终结果的列表
     * @param job  指定的任务函数
     * @param buffer_size 任务缓存和结果缓存的大小
     * @param wait_sleep_ms 如果无法从任务缓存中取到内容，就 sleep 一段时间
     */
    Worker(std::vector<TaskT_> &task_list, std::mutex &task_mtx,
           std::vector<ResultT_> &result_list, std::mutex &result_mtx,
           const JobType &job,
           size_t buffer_size, unsigned wait_sleep_ms)

            : BUFFER_SIZE(buffer_size), WAIT_SLEEP_MS(wait_sleep_ms),
              _task{task_list, task_mtx}, _result{result_list, result_mtx}, _job(job) {

        /* 为 buffer 预分配内存，以免 vector 频繁收缩内存 */
        _task_buffer.reserve(buffer_size);
        _result_buffer.reserve(buffer_size);
    }

    /* 让 worker 开始执行任务 */
    void inline start() {
        _thread = std::shared_ptr<std::thread>(new std::thread(&Worker::_thread_func, this));
    }

    /* 终止 worker 运行 */
    void inline stop() {
        _should_stop = true;
        _thread->join();
    }

private:
    /* worker 内线程运行的代码 */
    void _thread_func();


private:
    /* 总的任务列表，在多个 worker 之间共享，读取的时候需要加锁 */
    struct {
        std::vector<TaskT_> &task_list;
        std::mutex &task_mutex;
    } _task;

    /* 总的结果列表，在多个 worker 之间共享，读取的时候需要加锁 */
    struct {
        std::vector<ResultT_> &result_list;
        std::mutex &result_mutex;
    } _result;

    std::vector<TaskT_> _task_buffer{};            /* 为了减少读任务列表的次数 */
    std::vector<ResultT_> _result_buffer{};        /* 为了减少写结果列表的次数 */
    JobType _job;                                   /* 当前 workder 需要执行的任务 */

    std::shared_ptr<std::thread> _thread{nullptr};  /* worker 内部的线程 */
    bool _should_stop = false;                      /* worker 内的线程是否应该停止运行 */

};


template<class Task, class Result>
void Worker<Task, Result>::_thread_func() {
    auto _thread_id = std::this_thread::get_id();
    unsigned thread_id = *(unsigned int *) &(_thread_id);

    /* fixme：这个函数在多线程中调用，可能是不安全的 */
    spdlog::info("worker(thread id: {}) is running.", thread_id);

    while (!_should_stop) {     /* 通过设置这个变量，让线程结束 */
        /* 从任务列表取出任务放到任务 buffer 中，如果没有任务，就 sleep */
        if (_task.task_list.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_SLEEP_MS));
            continue;
        }
        {   /* 将 lock 限制在这个作用域里面 */
            std::lock_guard<std::mutex> lck(_task.task_mutex);
            auto cnt_task_alloc = std::min(_task.task_list.size(), BUFFER_SIZE);
            for (int i = 0; i < cnt_task_alloc; ++i) {
                _task_buffer.push_back(_task.task_list.back());
                _task.task_list.pop_back();
            }
        }

        /* 通过 job 来处理任务 */
        for (const auto &task : _task_buffer) {
            _result_buffer.push_back(_job(task));
        }
        _task_buffer.clear();

        /* 将结果 buffer 写入结果列表 */
        {
            std::lock_guard<std::mutex> lck(_result.result_mutex);
            _result.result_list.insert(_result.result_list.end(), _result_buffer.begin(), _result_buffer.end());
        }
        _result_buffer.clear();
    }

    spdlog::info("worker(thread id: {}) is over.", thread_id);
}


#endif //RENDER_DEBUG_TASK_H
