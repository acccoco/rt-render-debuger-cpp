
#define CATCH_CONFIG_MAIN

#include <catch2/catch.hpp>
#include "../bvh.h"
#include "../utils.h"
#include "../triangle.h"


// 遍历 BVH
void BVH_traverse(const std::shared_ptr<BVH> &node, const std::function<void(const BVH &)> &func) {
    assert(node);
    func(*node);
    if (node->lchild()) BVH_traverse(node->lchild(), func);
    if (node->rchild()) BVH_traverse(node->rchild(), func);
}


// NOTE：对照着那张图片来看这些测试用例
TEST_CASE("find k_th object, four triangle") {
    auto mat = std::shared_ptr<Material>(nullptr);

    auto t0 = std::make_shared<Triangle>(Eigen::Vector3f(4, 3, 3),
                                         Eigen::Vector3f(2, 5, 6),
                                         Eigen::Vector3f(0, 7, -3),
                                         mat);

    auto t1 = std::make_shared<Triangle>(Eigen::Vector3f(-4, 7, 2),
                                         Eigen::Vector3f(3, 4, -1),
                                         Eigen::Vector3f(0, -1, -2),
                                         mat);

    auto t2 = std::make_shared<Triangle>(Eigen::Vector3f(2, 4, 7),
                                         Eigen::Vector3f(-6, 5, 3),
                                         Eigen::Vector3f(4, -2, 7),
                                         mat);

    auto t3 = std::make_shared<Triangle>(Eigen::Vector3f(6, 7, 1),
                                         Eigen::Vector3f(-5, -3, -2),
                                         Eigen::Vector3f(0, -4, 5),
                                         mat);

    SECTION("手动计算的包围盒重心是否正确") {
        REQUIRE(t0->bounding_box().center() == Eigen::Vector3f(2, 5, 3 / 2.f));
        REQUIRE(t1->bounding_box().center() == Eigen::Vector3f(-1 / 2.f, 3, 0));
        REQUIRE(t2->bounding_box().center() == Eigen::Vector3f(-1, 3 / 2.f, 5));
        REQUIRE(t3->bounding_box().center() == Eigen::Vector3f(1 / 2.f, 3 / 2.f, 3 / 2.f));
    }

    const auto tris = std::vector<std::shared_ptr<Object>>{t0, t1, t2, t3};
    // 获取 tris 序列重心按特定方向排序的任意位
    auto get_order = [&tris](ExtensionDir dir, unsigned int order) {
        return std::get<1>(find_kth_obj(tris, order, dir));
    };

    SECTION("x direction") {
        REQUIRE(get_order(ExtensionDir::X, 0) == t2);
        REQUIRE(get_order(ExtensionDir::X, 1) == t1);
        REQUIRE(get_order(ExtensionDir::X, 2) == t3);
        REQUIRE(get_order(ExtensionDir::X, 3) == t0);
    }

    SECTION("y direction") {
        REQUIRE((get_order(ExtensionDir::Y, 0) == t2 || get_order(ExtensionDir::Y, 0) == t3));
        REQUIRE((get_order(ExtensionDir::Y, 1) == t2 || get_order(ExtensionDir::Y, 1) == t3));
        REQUIRE(get_order(ExtensionDir::Y, 2) == t1);
        REQUIRE(get_order(ExtensionDir::Y, 3) == t0);
    }

    SECTION("z direction") {
        REQUIRE(get_order(ExtensionDir::Z, 0) == t1);
        REQUIRE((get_order(ExtensionDir::Z, 1) == t0 || get_order(ExtensionDir::Z, 1) == t3));
        REQUIRE((get_order(ExtensionDir::Z, 2) == t0 || get_order(ExtensionDir::Z, 2) == t3));
        REQUIRE(get_order(ExtensionDir::Z, 3) == t2);
    }
}


TEST_CASE("build _bvh") {
    int obj_size = 4;
    std::vector<std::shared_ptr<Object>> objs(obj_size);

    auto mat = std::shared_ptr<Material>(nullptr);

    // 生成 object
    for (auto &obj : objs) {
        obj = std::make_shared<Triangle>(random_point_get(-5, 5),
                                         random_point_get(-5, 5),
                                         random_point_get(-5, 5),
                                         mat);
    }

    auto root = BVH::build(objs);

    SECTION("node count") {
        // 检查节点数量
        int cnt = 0;
        BVH_traverse(root, [&cnt](const BVH &) { cnt++; });
        REQUIRE(cnt == objs.size() * 2 - 1);
    }

    SECTION("只有叶子节点有 object，非叶子节点有两个 child") {
        BVH_traverse(root, [](const BVH &node) {
            if (node.object()) {
                REQUIRE(node.lchild() == nullptr);
                REQUIRE(node.rchild() == nullptr);
            } else {
                REQUIRE(node.lchild() != nullptr);
                REQUIRE(node.rchild() != nullptr);
            }
        });
    }
}

