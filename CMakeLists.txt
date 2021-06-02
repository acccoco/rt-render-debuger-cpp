cmake_minimum_required(VERSION 3.19)
project(render_debug)


# C++ 编译相关
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


# 路径定义
set(ROOT_DIR "${PROJECT_SOURCE_DIR}")
set(DB_PATH "${ROOT_DIR}/render.db")


# 添加路径宏定义
add_definitions(-DDB_PATH="${DB_PATH}")


# sqlite3
find_package(SQLite3)
if (NOT SQLite3_FOUND)
    message(FATAL_ERROR "can not find SQLite3")
endif()
message(STATUS "found SQLite3")
message(STATUS "libraries: ${SQLite3_LIBRARIES}")


# eigen
find_package(Eigen3)
if (NOT Eigen3_FOUND)
    message(FATAL_ERROR "can not find Eigen3")
endif()
message(STATUS "found Eigen3")
message(STATUS "include: ${EIGEN3_INCLUDE_DIR}")


# 子项目
add_subdirectory(render)
add_subdirectory(debugger)
add_subdirectory(demo)
