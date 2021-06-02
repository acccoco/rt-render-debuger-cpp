cmake_minimum_required(VERSION 3.19)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(render main.cpp bounding_box.h ray.h scene.h rt_render.h bvh.h triangle.h)


target_link_libraries(render
        ${SQLite3_LIBRARIES}
        )
