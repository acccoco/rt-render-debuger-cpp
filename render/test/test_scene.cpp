#define CATCH_CONFIG_MAIN

#include <string>
#include <fstream>

#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <catch2/catch.hpp>

#include "config.h"
#include "../scene.h"
#include "../utils.h"
#include "../triangle.h"


TEST_CASE("scene build 构建场景") {

    std::vector<std::string> file_path_list;
    for (auto name : {PATH_CORNELL_FLOOR,
                      PATH_CORNELL_LEFT,
                      PATH_CORNELL_LIGHT,
                      PATH_CORNELL_RIGHT,
                      PATH_CORNELL_SHORTBOX,
                      PATH_CORNELL_TALLBOX}) {
        file_path_list.emplace_back(name);
    }

    // 确保每个文件都存在
    for (auto &file_path : file_path_list) {
        std::fstream fs;
        fs.open(file_path, std::ios::in);
        REQUIRE(fs);
        fs.close();

        auto meshes = MeshTriangle::mesh_load(file_path);
        REQUIRE(meshes.size() == 1);
    }

    // 漫反射的物体
    auto floor = MeshTriangle::mesh_load(PATH_CORNELL_FLOOR)[0];
    auto left = MeshTriangle::mesh_load(PATH_CORNELL_LEFT)[0];
    auto right = MeshTriangle::mesh_load(PATH_CORNELL_RIGHT)[0];
    auto shortbox = MeshTriangle::mesh_load(PATH_CORNELL_SHORTBOX)[0];
    auto tallbox = MeshTriangle::mesh_load(PATH_CORNELL_TALLBOX)[0];
    floor->material().set_diffuse(color_cornel_white);
    shortbox->material().set_diffuse(color_cornel_white);
    tallbox->material().set_diffuse(color_cornel_white);
    left->material().set_diffuse(color_cornel_red);
    right->material().set_diffuse(color_cornel_green);

    // 发光的物体
    auto light = MeshTriangle::mesh_load(PATH_CORNELL_LIGHT)[0];
    light->material().set_emission(color_cornel_light);

    // 构建场景
    Scene scene(800, 600, 45.f, {0.f, 0.f, 1.f}, {0.f ,0.f, 0.f});
    scene.obj_add(floor);
    scene.obj_add(left);
    scene.obj_add(right);
    scene.obj_add(shortbox);
    scene.obj_add(tallbox);
    scene.obj_add(light);
    scene.build();


    // 确保发光体统计正确了
    REQUIRE((scene.emit().objs.size() == 1 && scene.emit().objs[0] == light));
    REQUIRE(EQUAL_F4(light->area(), scene.emit().total_area));
};
