cmake_minimum_required(VERSION 3.19)
project(render_debug)


############################################################
# 编译选项
############################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")


############################################################
# 系统的头文件和链接目录
############################################################
include_directories(/opt/homebrew/include)
link_directories(/opt/homebrew/lib)


############################################################
# 注入头文件
############################################################
# 测试相关的模型
set(ASSETS_DIR "${CMAKE_SOURCE_DIR}/assets/")
set(PATH_CORNELL "${CMAKE_SOURCE_DIR}/assets/cornell-box")
set(PATH_CORNELL_FLOOR "${PATH_CORNELL}/floor.obj")
set(PATH_CORNELL_LEFT "${PATH_CORNELL}/left.obj")
set(PATH_CORNELL_LIGHT "${PATH_CORNELL}/light.obj")
set(PATH_CORNELL_RIGHT "${PATH_CORNELL}/right.obj")
set(PATH_CORNELL_SHORTBOX "${PATH_CORNELL}/shortbox.obj")
set(PATH_CORNELL_TALLBOX "${PATH_CORNELL}/tallbox.obj")

# 存放渲染信息的数据库
set(DB_PATH "${CMAKE_SOURCE_DIR}/render.sqlite3")

# 渲染结果文件
set(RT_RES "${CMAKE_SOURCE_DIR}/rt.ppm")

# 注入头文件
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
find_package(assimp REQUIRED)


############################################################
# 子项目
############################################################
add_subdirectory(render)
add_subdirectory(demo)


############################################################
# cornel-box 的渲染
############################################################
add_executable(cornel-box main.cpp)
target_link_libraries(cornel-box PRIVATE render)
