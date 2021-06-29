
#include <string>

#include <fmt/format.h>
#include <spdlog/spdlog.h>

#ifndef CATCH_CONFIG_MAIN
#define CATCH_CONFIG_MAIN
#endif

#include <catch2/catch.hpp>

#include "config.h"
#include "utils.h"
#include "scene.h"
#include "triangle.h"


// =========================================================
// 三角形内采样，判断采样结果是否在三角形上
// =========================================================
TEST_CASE("在三角形内采样")
{
    auto mat = std::make_shared<Material>();

    LOOP(10) {
        auto tri = std::make_shared<Triangle>(random_point_get() * 100.f,
                                              random_point_get() * 100.f,
                                              random_point_get() * 100.f, mat);
        if (tri->area() <= 0)
            break;

        auto inter = tri->obj_sample(tri->area() * random_float_get());

        REQUIRE(inter.happened());
        REQUIRE(inter.normal().get() == tri->normal().get());

        float dot = (inter.pos() - tri->A()).dot(tri->normal().get());
        REQUIRE(std::abs(dot) < epsilon_4);
    }
}


// =========================================================
// 半球随机采样，判断采样结果是否在半球上
// =========================================================
TEST_CASE("半球均匀采样") {

    SECTION("采样分布在半球内") {
        Direction N({4, 5, 8});

        LOOP(10) {
            auto[pdf, vec] = Material::sample_himsphere_random(N);

            REQUIRE(std::abs(pdf - 0.5f / M_PI) < epsilon_4);
            REQUIRE(vec.get().dot(N.get()) > -epsilon_4);
            REQUIRE(std::abs(1.f - vec.get().norm()) < epsilon_4);
        }
    }
}


// =========================================================
// 这个测试用例只能保证 BVH 的采样基本可用
// =========================================================
TEST_CASE("在由三角形组成的 BVH 中采样") {
    // 随机生成三角形
    auto mat = std::make_shared<Material>();
    std::vector<std::shared_ptr<Object>> tris;
    LOOP(10) {
        auto tri = std::make_shared<Triangle>(random_point_get() * 100.f,
                                              random_point_get() * 100.f,
                                              random_point_get() * 100.f, mat);
        if (tri->area() <= 0.f) break;
        tris.push_back(tri);
    }

    // 构建BVH
    auto bvh_root = BVH::build(tris);

    // 随机地采样
    LOOP(10) {
        float area_threshold = random_float_get() * bvh_root->area();
        auto inter = bvh_root->sample_obj(area_threshold);

        REQUIRE(inter.happened());
    }
}


// =========================================================
// 在三角形模型上进行采样
// =========================================================
TEST_CASE("在 MeshTriangle 中采样") {

    auto path = [](const std::string &name) {
        return fmt::format("{}cornell-box/{}.obj", ASSETS_DIR, name);
    };

    // 导入模型文件
    auto floor = MeshTriangle::mesh_load(path("floor"))[0];
    auto shortbox = MeshTriangle::mesh_load(path("shortbox"))[0];

    // 随机采样
    SECTION("采样点位于包围盒内") {
        LOOP(10) {
            auto inter_floor = floor->obj_sample(floor->area() * random_float_get());
            auto inter_shortbox = shortbox->obj_sample(shortbox->area() * random_float_get());

            REQUIRE(floor->bounding_box().contain(inter_floor.pos()));
            REQUIRE(shortbox->bounding_box().contain(inter_shortbox.pos()));
        }
    }
}


// =========================================================
// 对场景中的所有光源进行采样
// =========================================================
TEST_CASE("对场景中的光源随机采样") {
    // 导入模型
    auto floor = MeshTriangle::mesh_load(PATH_CORNELL_FLOOR)[0];
    auto left = MeshTriangle::mesh_load(PATH_CORNELL_LEFT)[0];
    auto light = MeshTriangle::mesh_load(PATH_CORNELL_LIGHT)[0];

    auto right = MeshTriangle::mesh_load(PATH_CORNELL_RIGHT)[0];
    auto shortbox = MeshTriangle::mesh_load(PATH_CORNELL_SHORTBOX)[0];
    auto tallbox = MeshTriangle::mesh_load(PATH_CORNELL_TALLBOX)[0];
    right->mat()->set_emission(color_cornel_red);
    shortbox->mat()->set_emission(color_cornel_red);
    tallbox->mat()->set_emission(color_cornel_red);

    auto light_objs = std::vector<std::shared_ptr<MeshTriangle>>{right, shortbox, tallbox};

    // 建立场景
    Scene scene(800, 600, 45.f, {0.f, 0.f, 1.f}, {0.f ,0.f, 0.f});
    for (const auto &obj : {floor, left, light, right, shortbox, tallbox}) {
        scene.obj_add(obj);
    }
    scene.build();

    // 对光源随机采样
    LOOP(10) {
        auto [pdf, inter] = scene.sample_light();

        REQUIRE(EQUAL_F4(pdf, 1.f/ scene.emit().total_area));
        REQUIRE(inter.happened());
        REQUIRE(inter.mat()->is_emission());
    }
}
