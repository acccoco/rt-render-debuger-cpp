cmake_minimum_required(VERSION 3.19)
project(render)


############################################################
# target: render
############################################################
add_executable(${PROJECT_NAME}
        main.cpp
        src/bounding_box.cpp
        src/triangle.cpp)
target_link_libraries(render PRIVATE
        ${SQLite3_LIBRARIES}
        fmt::fmt
        spdlog::spdlog)


############################################################
# unit test
############################################################
add_executable(bounding_box
        unit_test/test_bounding_box.cpp
        src/bounding_box.cpp)
target_link_libraries(bounding_box PRIVATE
        Eigen3::Eigen
        fmt::fmt
        Catch2::Catch2
        spdlog::spdlog)
