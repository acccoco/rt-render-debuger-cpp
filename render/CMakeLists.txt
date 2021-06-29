cmake_minimum_required(VERSION 3.19)
project(render)


############################################################
# 所有的库文件
############################################################
set(LINKS
        SQLite::SQLite3
        fmt::fmt
        Eigen3::Eigen
        spdlog::spdlog
        ${ASSIMP_LIBRARIES})


############################################################
# 所有的源文件
############################################################
set(SOURCES
        src/bounding_box.cpp
        src/triangle.cpp
        src/bvh.cpp
        src/rt_render.cpp
        src/material.cpp
        src/scene.cpp)


############################################################
# target：编译为静态库
############################################################
add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LINKS})

# 指定头文件的目录，避免使用相对路径的方式
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})


############################################################
# target：单元测试
############################################################
set(tests
        bvh
        scene
        intersect
        sample
        mesh
        bounding_box
        task
        ray_trace
        render
        sqlite)

foreach (target ${tests})
    add_executable(test-${target} test/test_${target} ${SOURCES})
    target_link_libraries(test-${target} PRIVATE ${LINKS})
    target_include_directories(test-${target} PRIVATE ${PROJECT_SOURCE_DIR})
endforeach ()
