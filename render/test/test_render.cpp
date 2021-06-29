#ifndef CATCH_CONFIG_MAIN
#define CATCH_CONFIG_MAIN
#endif

#include <fmt/format.h>

#include <catch2/catch.hpp>
#include <string>

#include "triangle.h"
#include "utils.h"
#include "config.h"
#define private public
#include "rt_render.h"
#undef private


TEST_CASE("测试生成的任务是否正确")
{
    auto scene = std::make_shared<Scene>(2,
                                         2,
                                         45.f,
                                         Eigen::Vector3f(0.f, 0.f, 1.f),
                                         Eigen::Vector3f(100.f, 100.f, 0.f));
    RTRender::init(scene, 16);
    auto tasks = RTRender::_prepare_render_task(scene);

    REQUIRE(tasks.size() == 4);

    float value = (float)std::tan(45.f / 2.f / 180.f * M_PI) / 2.f;

    SECTION("所有的射线的原点")
    {
        for (auto &task : tasks)
        {
            REQUIRE((task.ray.origin() - Eigen::Vector3f(100.f, 100.f, 0.f)).norm() < epsilon_5);
        }
    }
    SECTION("左上角")
    {
        RTRender::RenderPixelTask task = tasks[0];
        REQUIRE(task.col == 0);
        REQUIRE(task.row == 0);
        REQUIRE((task.ray.direction().get() - Eigen::Vector3f(value, value, 1.f).normalized())
                    .norm() < epsilon_5);
    }
    SECTION("右上角")
    {
        RTRender::RenderPixelTask task = tasks[1];
        REQUIRE(task.col == 1);
        REQUIRE(task.row == 0);
        REQUIRE((task.ray.direction().get() - Eigen::Vector3f(-value, value, 1.f).normalized())
                    .norm() < epsilon_5);
    }
    SECTION("左下角")
    {
        RTRender::RenderPixelTask task = tasks[2];
        REQUIRE(task.col == 0);
        REQUIRE(task.row == 1);
        REQUIRE((task.ray.direction().get() - Eigen::Vector3f(value, -value, 1.f).normalized())
                    .norm() < epsilon_5);
    }
    SECTION("右下角")
    {
        RTRender::RenderPixelTask task = tasks[3];
        REQUIRE(task.col == 1);
        REQUIRE(task.row == 1);
        REQUIRE((task.ray.direction().get() - Eigen::Vector3f(-value, -value, 1.f).normalized())
                    .norm() < epsilon_5);
    }
}

TEST_CASE("写入 ppm 文件")
{
    // 构造颜色数据：从左到右红色增加，从上到下蓝色增加
    std::vector<std::array<unsigned char, 3>> framebuffer;
    unsigned char height = 200, width = 200;
    for (unsigned char row = 0; row < height; ++row)
    {
        for (unsigned char col = 0; col < width; ++col)
        {
            framebuffer.push_back({col, 0, row});
        }
    }

    // 写文件
    RTRender::write_to_file(framebuffer, TEST_RT_RES, width, height);
}

TEST_CASE("单线程小规模的计算任务")
{
    // 导入模型
    auto floor = MeshTriangle::mesh_load(PATH_CORNELL_FLOOR)[0];
    floor->mat()->set_diffuse(color_cornel_white);
    auto left = MeshTriangle::mesh_load(PATH_CORNELL_LEFT)[0];
    left->mat()->set_diffuse(color_cornel_red);
    auto right = MeshTriangle::mesh_load(PATH_CORNELL_RIGHT)[0];
    right->mat()->set_diffuse(color_cornel_green);
    auto tall_box = MeshTriangle::mesh_load(PATH_CORNELL_TALLBOX)[0];
    tall_box->mat()->set_diffuse(color_cornel_white);
    auto shot_box = MeshTriangle::mesh_load(PATH_CORNELL_SHORTBOX)[0];
    shot_box->mat()->set_diffuse(color_cornel_white);
    auto light = MeshTriangle::mesh_load(PATH_CORNELL_LIGHT)[0];
    light->mat()->set_emission(color_cornel_light);

    // 构建场景
    auto scene = std::make_shared<Scene>(20,
                                         20,
                                         40.f,
                                         Eigen::Vector3f{0.f, 0.f, 1.f},
                                         Eigen::Vector3f{278.f, 273.f, -800.f});
    scene->obj_add(floor);
    scene->obj_add(left);
    scene->obj_add(right);
    scene->obj_add(light);
    scene->obj_add(tall_box);
    scene->obj_add(shot_box);
    scene->build();

    // 进行渲染
    RTRender::init(scene, 1);
    RTRender::render_single_thread(DB_PATH);
    RTRender::write_to_file(RTRender::framebuffer,
                            RT_RES,
                            scene->screen_width(),
                            scene->screen_height());
}
