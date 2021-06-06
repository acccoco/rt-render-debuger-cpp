#include <iostream>
#include <stdexcept>
#include <sqlite3.h>

#include "config.h"

#include "material.h"
#include "intersection.h"
#include "object.h"
#include "bounding_box.h"
#include "ray.h"
#include "scene.h"
#include "utils.h"
#include "rt_render.h"
#include "triangle.h"


int main() {
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
    auto scene = std::make_shared<Scene>(200, 200, 40.f,
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
    auto start = std::chrono::system_clock::now();
    render.render(128);
    auto stop = std::chrono::system_clock::now();
    fmt::print("\nrender complete\ntime taken: {} hours, {} minutes, {} seconds\n",
               std::chrono::duration_cast<std::chrono::hours>(stop - start).count(),
               std::chrono::duration_cast<std::chrono::minutes>(stop - start).count(),
               std::chrono::duration_cast<std::chrono::seconds>(stop - start).count());

    return 0;
}

