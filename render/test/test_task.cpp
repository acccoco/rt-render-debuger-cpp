#include <string>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#ifndef CATCH_CONFIG_MAIN
#define CATCH_CONFIG_MAIN
#endif

#include <catch2/catch.hpp>

#include "../utils.h"
#include "../task.h"


// 用于测试的任务
struct TestTask {
    int task_id;
};

// 用于测试的结果类型
struct TestResult {
    int task_id;
    int result;
};


struct Foo {
    std::vector<TestResult> result_container;

    // 给 bee 执行的函数
    TestResult bee_job(const TestTask &task) {
        return TestResult{task.task_id, task.task_id + 1};
    }

    // 给 ant 执行的函数
    void ant_job(const TestResult &result) {
        result_container.push_back(result);
    }
};



TEST_CASE("task") {
    std::deque<TestTask> task_list;
    unsigned int total_task_cnt = 10000;
    for (int i = 0; i < total_task_cnt; ++i) {
        task_list.push_back(TestTask{i});
    }

    Foo foo;
    auto bee_job = [ObjectPtr = &foo](auto && PH1) { return ObjectPtr->bee_job(std::forward<decltype(PH1)>(PH1)); };
    auto ant_job = [ObjectPtr = &foo](auto && PH1) { ObjectPtr->ant_job(std::forward<decltype(PH1)>(PH1)); };
    Giraffe<TestTask, TestResult> giraffe(16, 100, 100, 50,
                                          bee_job, ant_job, task_list);

    giraffe.run();

    REQUIRE(foo.result_container.size() == total_task_cnt);
}
