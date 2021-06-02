cmake_minimum_required(VERSION 3.19)


add_executable(sql_demo sql.cpp)
target_link_libraries(sql_demo ${SQLite3_LIBRARIES})
