cmake_minimum_required(VERSION 3.19)
project(render_debug)


############################################################
# 编译选项
############################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


############################################################
# 数据库的路径
############################################################
set(DB_PATH "${CMAKE_SOURCE_DIR}/render.sqlite3")
configure_file(${CMAKE_SOURCE_DIR}/config.h.in ${CMAKE_BINARY_DIR}/config.h)
include_directories(${CMAKE_BINARY_DIR})


############################################################
# 第三方库
############################################################
find_package(SQLite3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Catch2 REQUIRED)
find_package(spdlog REQUIRED)


############################################################
# 子项目
############################################################
add_subdirectory(render)
add_subdirectory(debugger)
add_subdirectory(demo)
