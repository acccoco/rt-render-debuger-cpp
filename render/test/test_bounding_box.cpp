#define CATCH_CONFIG_MAIN

#include "bounding_box.h"

#include <string>

#include <catch2/catch.hpp>

TEST_CASE("构造函数")
{
    SECTION("无参构造函数")
    {
        BoundingBox box;

        auto min_f = std::numeric_limits<float>::lowest();
        auto max_f = std::numeric_limits<float>::max();

        REQUIRE(box.p_min == Eigen::Vector3f(max_f, max_f, max_f));
        REQUIRE(box.p_max == Eigen::Vector3f(min_f, min_f, min_f));
    }

    SECTION("通过一个点来创建 AABB 包围盒")
    {
        Eigen::Vector3f p1{1.f, -1.0f, 0.f};
        BoundingBox box1(p1);

        REQUIRE(box1.p_min == p1);
        REQUIRE(box1.p_max == p1);
    }

    SECTION("通过两个点来创建 AABB 包围盒")
    {
        Eigen::Vector3f p1{-1.f, 2.f, 3.f};
        Eigen::Vector3f p2{1.f, -1.f, 4.f};

        BoundingBox box3(p1, p2);

        REQUIRE(box3.p_min == Eigen::Vector3f(-1.f, -1.f, 3.f));
        REQUIRE(box3.p_max == Eigen::Vector3f(1.f, 2.f, 4.f));
    }
}

TEST_CASE("AABB 的与运算")
{
    const BoundingBox box(Eigen::Vector3f(1.f, 1.f, 1.f));

    SECTION("AABB 包围盒和一个点进行与运算")
    {
        BoundingBox box1 = box;

        Eigen::Vector3f p1{-1.f, -1.f, -1.f};
        box1.unionOp(p1);

        REQUIRE(box1.p_min == Eigen::Vector3f(-1.f, -1.f, -1.f));
        REQUIRE(box1.p_max == Eigen::Vector3f(1.f, 1.f, 1.f));
    }

    SECTION("AABB 包围盒之间进行与运算")
    {
        const BoundingBox &box1 = box;

        BoundingBox box2(Eigen::Vector3f(0.5f, 1.5f, 0.5f));
        box2.unionOp(Eigen::Vector3f(-3.f, -3.f, -3.f));

        auto box3 = BoundingBox::unionOp(box1, box2);

        REQUIRE(box3.p_min == Eigen::Vector3f(-3.f, -3.f, -3.f));
        REQUIRE(box3.p_max == Eigen::Vector3f(1.f, 1.5f, 1.f));
    }
}
