#ifndef CATCH_CONFIG_MAIN
#define CATCH_CONFIG_MAIN
#endif

#include <string>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <catch2/catch.hpp>

#include "config.h"
#include "../utils.h"
#include "../triangle.h"
#include "../rt_render.h"


// 测试生成的光线是否争取
TEST_CASE("task generate 测试生成的任务是否正确") {

    auto scene = std::make_shared<Scene>(2, 2, 45.f,
                                         Eigen::Vector3f(0.f, 0.f, 1.f),
                                         Eigen::Vector3f(100.f, 100.f, 0.f));
    RTRender render(scene);

    auto tasks = render.prepare_tasks();

    REQUIRE(tasks.size() == 4);

    float value = std::tan(45.f / 2.f / 180.f * M_PI) / 2.f;

    SECTION("所有的射线的原点") {
        for (auto &task : tasks) {
            REQUIRE((task.ray.origin() - Eigen::Vector3f(100.f, 100.f, 0.f)).norm() < epsilon_5);
        }
    }SECTION("左上角") {
        PixelTask task = tasks[0];
        REQUIRE(task.screen_x == 0);
        REQUIRE(task.screen_y == 0);
        REQUIRE((task.ray.direction().get() - Eigen::Vector3f(value, value, 1.f).normalized()).norm() < epsilon_5);
    }SECTION("右上角") {
        PixelTask task = tasks[1];
        REQUIRE(task.screen_x == 1);
        REQUIRE(task.screen_y == 0);
        REQUIRE((task.ray.direction().get() - Eigen::Vector3f(-value, value, 1.f).normalized()).norm() < epsilon_5);
    }SECTION("左下角") {
        PixelTask task = tasks[2];
        REQUIRE(task.screen_x == 0);
        REQUIRE(task.screen_y == 1);
        REQUIRE((task.ray.direction().get() - Eigen::Vector3f(value, -value, 1.f).normalized()).norm() < epsilon_5);
    }SECTION("右下角") {
        PixelTask task = tasks[3];
        REQUIRE(task.screen_x == 1);
        REQUIRE(task.screen_y == 1);
        REQUIRE((task.ray.direction().get() - Eigen::Vector3f(-value, -value, 1.f).normalized()).norm() < epsilon_5);
    }
}


TEST_CASE("write to file 文件写入") {
    // 构造颜色数据：从左到右红色增加，从上到下蓝色增加
    std::vector<std::vector<std::array<unsigned char, 3>>> buffer;
    for (unsigned char i = 0; i < 200; ++i) {
        std::vector<std::array<unsigned char, 3>> row;
        for (unsigned char j = 0; j < 200; ++j) {
            row.push_back({j, 0, i});
        }
        buffer.push_back(row);
    }

    // 写文件
    RTRender::write_to_file(buffer, TEST_RT_RES);
}


TEST_CASE("single thread 单线程小规模的计算任务") {
    // 导入模型
    auto floor = MeshTriangle::mesh_load(PATH_CORNELL_FLOOR)[0];
    floor->material().set_diffuse(color_cornel_white);
    auto left = MeshTriangle::mesh_load(PATH_CORNELL_LEFT)[0];
    left->material().set_diffuse(color_cornel_red);
    auto right = MeshTriangle::mesh_load(PATH_CORNELL_RIGHT)[0];
    right->material().set_diffuse(color_cornel_green);
    auto tall_box = MeshTriangle::mesh_load(PATH_CORNELL_TALLBOX)[0];
    tall_box->material().set_diffuse(color_cornel_white);
    auto shot_box = MeshTriangle::mesh_load(PATH_CORNELL_SHORTBOX)[0];
    shot_box->material().set_diffuse(color_cornel_white);
    auto light = MeshTriangle::mesh_load(PATH_CORNELL_LIGHT)[0];
    light->material().set_emission(color_cornel_light);

    // 构建场景
    auto scene = std::make_shared<Scene>(64, 64, 40.f,
                                         Eigen::Vector3f{0.f, 0.f, 1.f},
                                         Eigen::Vector3f{278.f, 273.f, -800.f});
    scene->obj_add(floor);
    scene->obj_add(left);
    scene->obj_add(right);
    scene->obj_add(light);
    scene->obj_add(tall_box);
    scene->obj_add(shot_box);
    scene->build();

    // 建立渲染器
    RTRender render(scene);

    // 进行渲染
    render.render_single_thread(1);
}
