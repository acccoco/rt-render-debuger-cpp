#ifndef RENDER_DEBUG_RAY_PATH_SERIALIZE_H
#define RENDER_DEBUG_RAY_PATH_SERIALIZE_H

#include <stdexcept>

#include <sqlite3.h>
#include <fmt/format.h>

#include "ray_path.h"


// ====
// 将光线的路径信息写入数据库，id 是一个全局变量，会自增
// ===

// 整理一下代码

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

    static inline sqlite3 *db = nullptr;
    static inline char *err_msg = nullptr;
};


/* 将 PathNode 信息写入到数据库 */
class NodeSerialize {
public:
    // todo 申请自增的 id 应该放到这里来
    /* 提取 PathNode 的信息，创建一个用于序列化的对象 */
    NodeSerialize(const PathNode &path_node, int64_t path_id)
            : id(path_id),
              Lo_x(path_node.Lo.x()), Lo_y(path_node.Lo.y()), Lo_z(path_node.Lo.z()),
              wo_x(path_node.wo.get().x()), wo_y(path_node.wo.get().y()), wo_z(path_node.wo.get().z()),
              inter_happened(path_node.inter.happened()),

              Li_light_x(path_node.from_light.Li_light.x()),
              Li_light_y(path_node.from_light.Li_light.y()),
              Li_light_z(path_node.from_light.Li_light.z()),
              wi_light_x(path_node.from_light.wi_light.get().x()),
              wi_light_y(path_node.from_light.wi_light.get().y()),
              wi_light_z(path_node.from_light.wi_light.get().z()),
              inter_light_happened(path_node.from_light.inter_light.happened()),

              wi_obj_x(path_node.from_obj.wi_obj.get().x()),
              wi_obj_y(path_node.from_obj.wi_obj.get().y()),
              wi_obj_z(path_node.from_obj.wi_obj.get().z()),
              RR(path_node.from_obj.RR),
              inter_obj_happened(path_node.from_obj.inter_obj.happened()) {}

    /* 将这个类写入到 sqlite 中 */
    static inline void write_to_sqlite(sqlite3 *db, const PathNode &node, int64_t path_id) {
        NodeSerialize n(node, path_id);

        /* 由于数据过多，为了直观，分三次构造字符串 */
        auto insert_str_part1 = fmt::format("'{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}'",
                                            n.id,
                                            n.Lo_x, n.Lo_y, n.Lo_z, n.wo_x, n.wo_y, n.wo_z,
                                            n.inter_happened);
        auto insert_str_part2 = fmt::format("'{}', '{}', '{}', '{}', '{}', '{}', '{}'",
                                            n.Li_light_x, n.Li_light_y, n.Li_light_z,
                                            n.wi_light_x, n.wi_light_y, n.wi_light_z,
                                            n.inter_light_happened);
        auto insert_str_part3 = fmt::format("'{}', '{}', '{}', '{}', '{}'",
                                            n.wi_obj_x, n.wi_obj_y, n.wi_obj_z,
                                            n.RR, n.inter_obj_happened);

        auto res = sqlite3_exec(db, fmt::format("INSERT INTO path_node VALUES ({}, {}, {})",
                                                insert_str_part1, insert_str_part2, insert_str_part3).c_str(),
                                nullptr, nullptr, &err_msg);

        /* 错误处理 */
        if (SQLITE_OK != res)
            throw std::runtime_error(fmt::format("fail to insert to path_node, err msg: {}", err_msg));
    }

    /* 将一个路径节点写入数据库，id 会自动增加 */
    static inline void insert_node(sqlite3 *db, const PathNode &node, int64_t path_id) {

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
                obj.RR, obj.inter_obj.obj() && obj.inter_obj.obj()->material().is_emission());

        /* 执行 INSERT */
        auto res = sqlite3_exec(db, fmt::format("INSERT INTO node VALUES ({}, {}, {}, {})",
                                                path_id, str_part_out, str_part_light, str_part_obj).c_str(),
                                nullptr, nullptr, &err_msg);

        /* 错误处理 */
        if (SQLITE_OK != res)
            throw std::runtime_error(fmt::format("fail to insert to path_node, err msg: {}", err_msg));
    }

    /* 构造一个在进程期间不会重复的 id */
    static inline int64_t get_id() {
        static int64_t static_id = 0;
        return static_id++;
    }

    /* 存放错误信息 */
    static inline char *err_msg = nullptr;

private:
    // todo 这些可以全部去掉了，没有必要中间加一层
    int64_t id;                                     /* 路径节点的 id */
    float Lo_x, Lo_y, Lo_z;                         /* 出射光线的辐照度 */
    float wo_x, wo_y, wo_z;                         /* 出射光线的方向 */
    bool inter_happened;                            /* 是否和物体发生了相交 */

    float Li_light_x, Li_light_y, Li_light_z;       /* 来自光源的光线，辐照度 */
    float wi_light_x, wi_light_y, wi_light_z;       /* 来自光源的入射光方向 */
    bool inter_light_happened;                      /* 是否和光源发生了相交 */

    float wi_obj_x, wi_obj_y, wi_obj_z;             /* 来自物体的入射光方向 */
    float RR;                                       /* 俄罗斯轮盘赌的概率 */
    bool inter_obj_happened;                        /* 是否和物体发生了相交 */
};


/* 一条光线路径 */
class PathSerialize {
public:

    /* 将一条光路及其节点写入到数据库 */
    static void write_to_sqlite(sqlite3 *db, int row, int col, const std::deque<PathNode> &path) {
        PathSerialize path_ser;
        path_ser.row = row;
        path_ser.col = col;

        /* 将光线 path 的每一个节点存放到数据库里面 */
        for (const auto &path_node : path) {
            ++path_ser.path_node_cnt;
            auto id = NodeSerialize::get_id();
            // NodeSerialize::write_to_sqlite(db, path_node, id);
            NodeSerialize::insert_node(db, path_node, id);
            path_ser.path_node_ids.push_back(id);
        }

        /* 将光线的路径信息写入数据库 */
        insert_path(db, path_ser);
    }

    /* 将光路信息写入数据库 */
    static void insert_path(sqlite3 *db, const PathSerialize &path) {
        /* 将 PathNode 的 id 列表转换为字符串 */
        std::string ids = "";
        for (auto id: path.path_node_ids) {
            ids += fmt::format("{} ", id);
        }

        /* 写入数据库 */
        auto res = sqlite3_exec(db, fmt::format("INSERT INTO path VALUES ('{}', '{}', '{}', '{}')",
                                                path.row, path.col, path.path_node_cnt, ids).c_str(),
                                nullptr, nullptr, &err_msg);

        /* 错误处理 */
        if (SQLITE_OK != res)
            std::runtime_error(fmt::format("fail to insert into path, err msg: {}", err_msg));
    }

    static inline char *err_msg = nullptr;

private:
    int row, col;                               /* 当前光路对应的屏幕像素坐标 */
    int path_node_cnt = 0;                      /* 光路有多少个节点 */
    std::vector<int64_t> path_node_ids{};       /* 光路各个节点的 id，从摄像机出发 */
};

#endif //RENDER_DEBUG_RAY_PATH_SERIALIZE_H
