cmake_minimum_required(VERSION 3.19)
project(demo)


############################################################
# target: demo
############################################################
add_executable(${PROJECT_NAME} sql.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE
        SQLite::SQLite3)

