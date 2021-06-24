#include <string>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#ifndef CATCH_CONFIG_MAIN
#define CATCH_CONFIG_MAIN
#endif

#include <catch2/catch.hpp>

#include "../utils.h"
#include "../task.h"


TEST_CASE("worker") {
    /* 给 worker 执行的任务类型 */
    struct Task {
        int task_id;
    };

    /* 任务处理后生成的结果 */
    struct Result {
        int task_id;
        int res;
    };

    /* 处理任务的函数 */
    auto job = [](const Task &task) {
        return Result{task.task_id, task.task_id + 1};
    };

    /* 任务列表 */
    int task_cnt = 10000;
    std::vector<Task> task_list(task_cnt);
    for (int i = 0; i < task_cnt; ++i)
        task_list[i] = Task{i};
    std::mutex task_mtx;

    /* 结果列表 */
    std::vector<Result> result_list;
    std::mutex result_mtx;

    /* 初始化 worker */
    std::vector<Worker<Task, Result>>
            workers(32, Worker<Task, Result>(
            task_list, task_mtx, result_list, result_mtx, job,
            1, 50));

    /* 让 worker 运行 */
    for (auto &worker : workers)
        worker.start();

    /* 回收 worker */
    while (result_list.size() < task_cnt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    for (auto &worker : workers)
        worker.stop();

    /* 验证结果 */
    for (int i = 0; i < task_cnt; ++i) {
        REQUIRE(result_list[i].res == result_list[i].task_id + 1);
    }
}

