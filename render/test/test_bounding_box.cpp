#define CATCH_CONFIG_MAIN

#include "../bounding_box.h"
#include <string>
#include <iostream>
#include <fmt/format.h>
#include <catch2/catch.hpp>

TEST_CASE("constructor")
{
    SECTION("constructor no param")
    {
        BoundingBox box;

        auto min_f = std::numeric_limits<float>::lowest();
        auto max_f = std::numeric_limits<float>::max();

        REQUIRE(box.p_min == Eigen::Vector3f(max_f, max_f, max_f));
        REQUIRE(box.p_max == Eigen::Vector3f(min_f, min_f, min_f));
    }

    SECTION("constructor using 1 point")
    {
        Eigen::Vector3f p1{1.f, -1.0f, 0.f};
        BoundingBox box1(p1);

        REQUIRE(box1.p_min == p1);
        REQUIRE(box1.p_max == p1);
    }

    SECTION("constructor using 2 point")
    {
        Eigen::Vector3f p1{-1.f, 2.f, 3.f};
        Eigen::Vector3f p2{1.f, -1.f, 4.f};

        BoundingBox box3(p1, p2);

        REQUIRE(box3.p_min == Eigen::Vector3f(-1.f, -1.f, 3.f));
        REQUIRE(box3.p_max == Eigen::Vector3f(1.f, 2.f, 4.f));
    }
}

TEST_CASE("union operation")
{
    const BoundingBox box(Eigen::Vector3f(1.f, 1.f, 1.f));

    SECTION("box union point")
    {
        BoundingBox box1 = box;

        Eigen::Vector3f p1{-1.f, -1.f, -1.f};
        box1.union_(p1);

        REQUIRE(box1.p_min == Eigen::Vector3f(-1.f, -1.f, -1.f));
        REQUIRE(box1.p_max == Eigen::Vector3f(1.f, 1.f, 1.f));
    }

    SECTION("box union box")
    {
        const BoundingBox &box1 = box;

        BoundingBox box2(Eigen::Vector3f(0.5f, 1.5f, 0.5f));
        box2.union_(Eigen::Vector3f(-3.f, -3.f, -3.f));

        auto box3 = BoundingBox::union_(box1, box2);

        REQUIRE(box3.p_min == Eigen::Vector3f(-3.f, -3.f, -3.f));
        REQUIRE(box3.p_max == Eigen::Vector3f(1.f, 1.5f, 1.f));
    }
}

