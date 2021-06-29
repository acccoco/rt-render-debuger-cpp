#ifndef RENDER_DEBUG_RAY_PATH_SERIALIZE_H
#define RENDER_DEBUG_RAY_PATH_SERIALIZE_H

#include <stdexcept>

#include <sqlite3.h>
#include <fmt/format.h>

#include "ray_path.h"


/**
 * 将数据库常见的一些操作放在这个类里面
 *
 * 基本使用方法
 *  DB::init();
 *  ...
 *  DB::close_db();
 *
 * 使用事务来批量读写
 *  DB::transaction_begin();
 *  // 进行读写
 *  DB::transaction_commit();
 */
class DB {
public:
    /* 初始化数据库连接 */
    static inline void init_db(const std::string &db_path) {
        auto res = sqlite3_open(db_path.c_str(), &db);
        if (SQLITE_OK != res)
            throw std::runtime_error(fmt::format("fail to open sqlite3, path: {}", db_path));
    }

    /* 断开数据库的连接 */
    static inline void close_db() {
        sqlite3_close(db);
        db = nullptr;
    }

    /* 开始事务 */
    static inline void transaction_begin() {
        auto res = sqlite3_exec(db, "BEGIN;", nullptr, nullptr, &err_msg);
        if (SQLITE_OK != res)
            throw std::runtime_error(fmt::format("fail to begin transaction, err msg: {}", err_msg));
    }

    /* 提交事务 */
    static inline void transaction_commit() {
        auto res = sqlite3_exec(db, "COMMIT;", nullptr, nullptr, &err_msg);
        if (SQLITE_OK != res)
            throw std::runtime_error(fmt::format("fail to begin transaction, err msg: {}", err_msg));
    }

    /**
     * 构造一个在进程期间不会重复的 id
     * @注 并不是线程安全的
     */
    static inline int64_t get_id() {
        static int64_t static_id = 0;
        return static_id++;
    }

    static inline sqlite3 *db = nullptr;            /* 数据库的连接对象 */
    static inline char *err_msg = nullptr;          /* 错误信息 */
};


/* 将 PathNode 信息写入到数据库 */
class NodeSerialize {
public:

    static inline const char *TABLE = "node";

    /* 删除数据库中的这张表 */
    static inline void deleteTable(sqlite3 *db) {
        auto res = sqlite3_exec(db, fmt::format("DELETE FROM {}", TABLE).c_str(),
                                nullptr, nullptr, &err_msg);
        if (SQLITE_OK != res) {
            throw std::runtime_error(fmt::format("fail to delete table: {}", TABLE));
        }
    }

    /* 将一个路径节点写入数据库，id 会自动增加 */
    static inline void insertNode(sqlite3 *db, const PathNode &node, int64_t path_id) {

        /* 出射光线部分 */
        auto str_part_out = fmt::format(
                "'{}', '{}', '{}', "
                "'{}', '{}', '{}', "
                "'{}', '{}', '{}', "
                "'{:d}', "
                "'{}', '{}', '{}'",
                node.Lo.x(), node.Lo.y(), node.Lo.z(),
                node.wo.get().x(), node.wo.get().y(), node.wo.get().z(),
                node.pos_out.x(), node.pos_out.y(), node.pos_out.z(),
                node.inter.happened(),
                node.inter.pos().x(), node.inter.pos().y(), node.inter.pos().z());

        /* 和光源相关的部分 */
        auto &light = node.from_light;
        auto str_part_light = fmt::format(
                "'{}', '{}', '{}', "
                "'{}', '{}', '{}', "
                "'{:d}', "
                "'{}', '{}', '{}'",
                light.Li_light.x(), light.Li_light.y(), light.Li_light.z(),
                light.wi_light.get().x(), light.wi_light.get().y(), light.wi_light.get().z(),
                light.inter_light.happened(),
                light.inter_light.pos().x(), light.inter_light.pos().y(), light.inter_light.pos().z());

        /* 和物体相关的部分 */
        auto &obj = node.from_obj;
        auto str_part_obj = fmt::format(
                "'{}', '{}', '{}', "
                "'{}', '{}', '{}', "
                "'{:d}', "
                "'{}', '{}', '{}', "
                "'{}', '{:d}'",
                obj.Li_obj.x(), obj.Li_obj.y(), obj.Li_obj.z(),
                obj.wi_obj.get().x(), obj.wi_obj.get().y(), obj.wi_obj.get().z(),
                obj.inter_obj.happened(),
                obj.inter_obj.pos().x(), obj.inter_obj.pos().y(), obj.inter_obj.pos().z(),
                obj.RR, obj.inter_obj.mat() && obj.inter_obj.mat()->is_emission());

        /* 执行 INSERT */
        auto res = sqlite3_exec(db, fmt::format("INSERT INTO {} VALUES ({}, {}, {}, {})",
                                                TABLE, path_id, str_part_out, str_part_light, str_part_obj).c_str(),
                                nullptr, nullptr, &err_msg);

        /* 错误处理 */
        if (SQLITE_OK != res)
            throw std::runtime_error(fmt::format("fail to insert to path_node, err msg: {}", err_msg));
    }

    /* 存放错误信息 */
    static inline char *err_msg = nullptr;
};


/* 一条光线路径 */
class PathSerialize {
public:

    static inline const char *TABLE = "path";

    /* 删除数据库中的这张表 */
    static inline void deleteTable(sqlite3 *db) {
        auto res = sqlite3_exec(db, fmt::format("DELETE FROM {}", TABLE).c_str(),
                                nullptr, nullptr, &err_msg);
        if (SQLITE_OK != res) {
            throw std::runtime_error(fmt::format("fail to delete table: {}", TABLE));
        }
    }

    /**
     * 将一条光路及其节点写入到数据库
     * 光路信息包括：row，col，node-cnt，ids
     * 光路节点信息为：各种 Radiance，wi，wo 等
     */
    static void insertPath(sqlite3 *db, int row, int col, const std::deque<PathNode> &path) {
        PathSerialize path_ser;
        path_ser.row = row;
        path_ser.col = col;

        /* 将光线 path 的每一个节点存放到数据库里面 */
        for (const auto &path_node : path) {
            ++path_ser.path_node_cnt;
            auto id = DB::get_id();
            NodeSerialize::insertNode(db, path_node, id);
            path_ser.path_node_ids.push_back(id);
        }

        /* 将光线的路径信息写入数据库 */
        insertPathInfo(db, path_ser);
    }


private:
    /**
     * 将光路信息写入数据库
     * 光路信息包括：row，col，node-cnt，ids
     */
    static void insertPathInfo(sqlite3 *db, const PathSerialize &path) {
        /**
         * 将 PathNode 的 id 列表转换为字符串
         * 如：将 [122, 233, 445] 转化为； "122, 233, 455"
         */
        std::string ids;
        for (auto id: path.path_node_ids) {
            ids += fmt::format("{} ", id);
        }

        /* 写入数据库 */
        auto res = sqlite3_exec(db, fmt::format("INSERT INTO {} VALUES ('{}', '{}', '{}', '{}')",
                                                TABLE, path.row, path.col, path.path_node_cnt, ids).c_str(),
                                nullptr, nullptr, &err_msg);

        /* 错误处理 */
        if (SQLITE_OK != res)
            throw std::runtime_error(fmt::format("fail to insert into path, err msg: {}", err_msg));
    }

    static inline char *err_msg = nullptr;          /* 存放错误信息 */

private:
    int row{}, col{};                               /* 当前光路对应的屏幕像素坐标 */
    int path_node_cnt = 0;                          /* 光路有多少个节点 */
    std::vector<int64_t> path_node_ids{};           /* 光路各个节点的 id，从摄像机出发 */
};

#endif //RENDER_DEBUG_RAY_PATH_SERIALIZE_H
