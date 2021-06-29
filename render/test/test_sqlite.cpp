#ifndef CATCH_CONFIG_MAIN
#define CATCH_CONFIG_MAIN
#endif

#include <string>
#include <chrono>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <catch2/catch.hpp>

#include "config.h"
#include "../ray_path_serialize.h"
#include "../rt_render.h"

// =============================================================================
// 通过以下测试可以得出结论：在批量写入时，开启事务可以大幅度提高写入性能，大约为 70 倍
// =============================================================================


TEST_CASE("插入一条数据到 node 表中") {
    DB::init_db(DB_PATH);

    PathNode node;
    NodeSerialize::deleteTable(DB::db);
    NodeSerialize::insertNode(DB::db, node, 12);

    DB::close_db();
}

size_t RECORD_CNT = 20000;

TEST_CASE("批量插入到 node 表中 -无事务-计入构造str的时间") {
    DB::init_db(DB_PATH);
    PathNode node;

    /* 删除原来的数据 */
    NodeSerialize::deleteTable(DB::db);

    /* 开始批量插入 */
    auto start_time = std::chrono::system_clock::now();
    for (int i = 0; i < RECORD_CNT; ++i) {
        NodeSerialize::insertNode(DB::db, node, i);
    }
    auto end_time = std::chrono::system_clock::now();

    DB::close_db();

    /* 统计时间 */
    auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    fmt::print("批量插入-无事务-计入构造str的时间: {}ms", delta_time.count());
}

TEST_CASE("批量插入到 node 表中 -使用事务-计入构造str的时间") {
    DB::init_db(DB_PATH);
    PathNode node;

    /* 删除原来的数据 */
    NodeSerialize::deleteTable(DB::db);

    /* 开启事务并批量写入 */
    DB::transaction_begin();
    auto start_time = std::chrono::system_clock::now();
    for (int i = 0; i < RECORD_CNT; ++i) {
        NodeSerialize::insertNode(DB::db, node, i);
    }
    auto end_time = std::chrono::system_clock::now();
    DB::transaction_commit();

    DB::close_db();

    /* 统计时间 */
    auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    fmt::print("批量插入-使用事务-计入构造str的时间: {}ms", delta_time.count());
}


TEST_CASE ("写入一条记录到 path 表中") {
    /* 连接数据库 */
    DB::init_db(DB_PATH);

    /* 清除旧数据 */
    NodeSerialize::deleteTable(DB::db);
    PathSerialize::deleteTable(DB::db);

    /* 构造光路信息 */
    std::deque<PathNode> path(10, PathNode());

    /* 写入 */
    PathSerialize::insertPath(DB::db, 40, 50, path);

    /* 关闭数据库 */
    DB::close_db();
}


TEST_CASE("写入一个像素对应的多条光路（包括 node 和 path）-使用事务") {
    /* 连接数据库 */
    DB::init_db(DB_PATH);

    /* 清除旧数据 */
    NodeSerialize::deleteTable(DB::db);
    PathSerialize::deleteTable(DB::db);

    /* 构造光路信息 */
    RTRender::RenderPixelResult res;
    res.row = 400;
    res.col = 300;
    int spp = 16;
    for (int i = 0; i < spp; ++i) {
        res.path_list.emplace_back(10 + i, PathNode());
    }

    /* 写入 */
    DB::transaction_begin();
    for (auto &path : res.path_list) {
        PathSerialize::insertPath(DB::db, res.row, res.col, path);
    }
    DB::transaction_commit();

    /* 关闭数据库 */
    DB::close_db();
}

