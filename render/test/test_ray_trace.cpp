#ifndef CATCH_CONFIG_MAIN
#define CATCH_CONFIG_MAIN
#endif

#include <string>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <catch2/catch.hpp>

#include "config.h"
#define private public
#include "../rt_render.h"
#undef private
#include "../utils.h"
#include "../triangle.h"


TEST_CASE("one ray 场景中一根光线的弹射情况") {
    // 导入模型
    auto floor = MeshTriangle::mesh_load(PATH_CORNELL_FLOOR)[0];
    auto left = MeshTriangle::mesh_load(PATH_CORNELL_LEFT)[0];
    auto right = MeshTriangle::mesh_load(PATH_CORNELL_RIGHT)[0];
    auto light = MeshTriangle::mesh_load(PATH_CORNELL_LIGHT)[0];
    light->material().set_emission(color_cornel_light);

    // 构建场景
    auto scene = std::make_shared<Scene>(800, 600, 45.f, Eigen::Vector3f{0.f, 0.f, 1.f},
                                         Eigen::Vector3f{0.f, 0.f, 0.f});
    scene->obj_add(floor);
    scene->obj_add(left);
    scene->obj_add(right);
    scene->obj_add(light);
    scene->build();

    // 建立 Render 对象
    RTRender::init(scene, 1);

    // 投射光线
    Ray ray({250.f, 250.f, 0.f}, {0.357f, 0.257f, 1.f});
    std::deque<PathNode> path = RTRender::cast_ray(ray);
    fmt::print("path node cnt: {}\n", path.size());
}

TEST_CASE("verify path 验证路径信息是否完备") {
    // 导入模型
    auto floor = MeshTriangle::mesh_load(PATH_CORNELL_FLOOR)[0];
    auto left = MeshTriangle::mesh_load(PATH_CORNELL_LEFT)[0];
    auto right = MeshTriangle::mesh_load(PATH_CORNELL_RIGHT)[0];
    auto light = MeshTriangle::mesh_load(PATH_CORNELL_LIGHT)[0];
    light->material().set_emission(color_cornel_light);

    // 构建场景
    auto scene = std::make_shared<Scene>(800, 600, 45.f, Eigen::Vector3f{0.f, 0.f, 1.f},
                                         Eigen::Vector3f{0.f, 0.f, 0.f});
    scene->obj_add(floor);
    scene->obj_add(left);
    scene->obj_add(right);
    scene->obj_add(light);
    scene->build();

    // 建立 Render 对象
    RTRender::init(scene, 1);

    // 投射光线
    Ray ray({250.f, 250.f, 0.f}, {0.357f, 0.257f, 1.f});
    std::deque<PathNode> path = RTRender::cast_ray(ray);
}
