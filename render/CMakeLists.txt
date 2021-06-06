cmake_minimum_required(VERSION 3.19)
project(render)



############################################################
# 所有的链接目标
############################################################
set(LINKS
        SQLite::SQLite3
        fmt::fmt
        Eigen3::Eigen
        spdlog::spdlog
        ${ASSIMP_LIBRARIES}
        )


############################################################
# 所有的源文件
############################################################
set(SOURCES
        src/bounding_box.cpp
        src/triangle.cpp
        src/intersection.cpp
        src/bvh.cpp
        src/rt_render.cpp
        src/material.cpp
        src/scene.cpp
        src/object.cpp
        )


############################################################
# target: render
############################################################
add_executable(${PROJECT_NAME} main.cpp ${SOURCES})
target_link_libraries(render PRIVATE ${LINKS})


############################################################
# unit test
############################################################
function(test_target target_name source)
    add_executable(${target_name} ${source} ${SOURCES})
    target_link_libraries(${target_name} PRIVATE ${LINKS})
endfunction()


test_target(test_bvh test/test_bvh.cpp)
test_target(test_scene test/test_scene.cpp)
test_target(test_intersect test/test_intersect.cpp)
test_target(test_sample test/test_sample.cpp)
test_target(test_mesh test/test_mesh.cpp)
test_target(test_bounding_box test/test_bounding_box.cpp)
test_target(test_task test/test_task.cpp)
test_target(test_ray_trace test/test_ray_trace.cpp)
test_target(test_render test/test_render.cpp)