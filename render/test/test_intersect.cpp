
#include <string>

#include <fmt/format.h>
#ifndef CATCH_CONFIG_MAIN
#define CATCH_CONFIG_MAIN
#endif
#include <catch2/catch.hpp>

#include "../bvh.h"
#include "../utils.h"
#include "../triangle.h"


TEST_CASE("AABB-interseciton 包围盒-计算交点")
{
    BoundingBox box1(Eigen::Vector3f(1.f, 1.f, 1.f),
                     Eigen::Vector3f(-1.f, -1.f, -1.f));
    SECTION("parall")
    {
        Eigen::Vector3f ori1(0.f, 0.f, 0.f);
        Direction dir1({0.f, 1.f, 1.f});
        Ray ray1(ori1, dir1);
        REQUIRE(box1.is_intersect(ray1) == true);

        Eigen::Vector3f ori2(2.f, 0.f, 0.f);
        Ray ray2(ori2, dir1);
        REQUIRE(box1.is_intersect(ray2) == false);
    }

    SECTION("face AABB 没有体积的包围盒") {
        // 位于 X-Y 平面的包围盒，没有体积
        BoundingBox box2(Eigen::Vector3f(1.f, 0.f, 0.f),
                         Eigen::Vector3f(0.f, 1.f, 0.f));
        box2.union_(Eigen::Vector3f(0.f, 0.f, 0.f));

        REQUIRE(box2.p_max.z() == box2.p_min.z());

        Eigen::Vector3f orig = Eigen::Vector3f{0.f, 0.f, 3.f};
        Direction dir(Eigen::Vector3f{0.5f, 0.5f, 0.f} - orig);
        Ray ray(orig, dir);
        REQUIRE(box2.is_intersect(ray));
    }
}


TEST_CASE("triangle intersect 三角形计算交点")
{
    auto mat = std::shared_ptr<Material>();

    LOOP(10) {
        auto tri = std::make_shared<Triangle>(random_point_get() * 100.f,
                                              random_point_get() * 100.f,
                                              random_point_get() * 100.f, mat);
        // 在三角形内随机取一点
        auto bary = random_point_get();
        float total = bary.x() + bary.y() + bary.z();
        bary /= total;
        Eigen::Vector3f inter_pos = tri->A() * bary.x() + tri->B() * bary.y() + tri->C() * bary.z();

        auto orig = Eigen::Vector3f(0.f, 0.f, 0.f);
        Ray ray(orig, inter_pos - orig);

        auto inter = Triangle::intersect(tri, ray);

        REQUIRE(inter.happened());
        REQUIRE(inter.obj() == std::static_pointer_cast<Object>(tri));
        REQUIRE(inter.normal().get() == tri->normal().get());

        // fixme: 计算三角形交点的算法误差比较大
        float delta = (inter.pos() - inter_pos).norm();
        REQUIRE(delta < epsilon_1);
    }
}


TEST_CASE("_bvh intersect BVH 交点计算") {
    auto mat = std::shared_ptr<Material>(nullptr);

    // 三个三角形，可以对照图片
    auto t1 = std::make_shared<Triangle>(Eigen::Vector3f(0.3, 1.6, 0.4),
                                         Eigen::Vector3f(-0.7, 0.4, 4.2),
                                         Eigen::Vector3f(2.1, -3.2, 3.2),
                                         mat);
    Eigen::Vector3f p_in_1{0.62, -0.8, 3.04};
    auto t2 = std::make_shared<Triangle>(Eigen::Vector3f(1, 1.5, 0.3),
                                         Eigen::Vector3f(0.2, -3, 2.7),
                                         Eigen::Vector3f(3, -1.2, 0.4),
                                         mat);
    Eigen::Vector3f p_in_2{2.12, -0.57, 0.6};
    auto t3 = std::make_shared<Triangle>(Eigen::Vector3f(5, 1, 0),
                                         Eigen::Vector3f(4.5, -1, 0.8),
                                         Eigen::Vector3f(5.5, -1, -1),
                                         mat);
    Eigen::Vector3f p_in_3{4.9, -0.6, 0.1};

    // 构造 BVH
    auto objs = std::vector<std::shared_ptr<Object>>{t1, t2, t3};
    auto root = BVH::build(objs);

    auto delta = std::numeric_limits<float>::epsilon() * 10;

    SECTION("intersect nothing") {
        Ray ray1({7, 0, 0}, {1, 0, 0});
        REQUIRE(root->intersect(ray1).happened() == false);
    }

    SECTION("intersect with triangle 3") {
        Eigen::Vector3f origin{7, 0, 0};
        Eigen::Vector3f direction = p_in_3 - origin;
        direction.normalize();
        Ray ray1(origin, direction);

        auto inter = root->intersect(ray1);
        REQUIRE(inter.happened());
        REQUIRE(inter.obj() == t3);
        auto hit_delta = inter.pos() - p_in_3;
        for (auto i : {hit_delta.x(), hit_delta.y(), hit_delta.z()}) {
            REQUIRE(std::abs(i) < delta);
        }
    }

    SECTION("intersect with triangle 1") {
        Eigen::Vector3f origin{-3, 1, -1};
        Eigen::Vector3f direction = p_in_1 - origin;
        Ray ray1(origin, direction);

        auto inter = root->intersect(ray1);
        REQUIRE(inter.happened());
        REQUIRE(inter.obj() == t1);
        Eigen::Vector3f hit_delta = inter.pos() - p_in_1;
        for (auto i : {hit_delta.x(), hit_delta.y(), hit_delta.z()}) {
            REQUIRE(std::abs(i) < delta);
        }
    }
}


TEST_CASE("mesh triangle intersect 三角形模型相交计算") {

    // 构建 MeshTriangle
    auto tris = std::vector<std::shared_ptr<Object>>();
    auto mat = std::make_shared<Material>();

    LOOP(10) {
        auto tri = std::make_shared<Triangle>(random_point_get() * 100.f,
                                              random_point_get() * 100.f,
                                              random_point_get() * 100.f, mat);

        if (tri->area() <= 0.f)
            break;
        tris.push_back(tri);
    }
    auto bvh_root = BVH::build(tris);
    auto mesh = std::make_shared<MeshTriangle>(mat, bvh_root);

    LOOP(10) {
        // 随机选一个交点
        auto bary = random_point_get();
        auto total = bary.x() + bary.y() + bary.z();
        bary /= total;
        auto tri_idx = (unsigned int) ((float) tris.size() * random_float_get());
        const auto &tri = std::dynamic_pointer_cast<Triangle>(tris[tri_idx]);
        Eigen::Vector3f target_pos = bary.x() * tri->A() + bary.y() * tri->B() + bary.z() * tri->C();

        // 生成光线
        auto orig = random_point_get();
        Ray ray{orig, target_pos - orig};

        // 计算交点
        auto inter = MeshTriangle::intersec(mesh, ray);
        REQUIRE(inter.happened());

        // 交点的位置比 target_pos 更近
        float target_dis = (target_pos - orig).norm();
        float inter_dis = (inter.pos() - orig).norm();
        REQUIRE(target_dis - inter_dis >= -0.02f * target_dis);
    }
}

