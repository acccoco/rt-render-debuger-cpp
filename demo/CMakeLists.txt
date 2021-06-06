cmake_minimum_required(VERSION 3.19)
project(demo)


set(LINKS
        SQLite::SQLite3
        fmt::fmt
        )


############################################################
# target: demo
############################################################
add_executable(demo_sql sql.cpp)
target_link_libraries(demo_sql PRIVATE ${LINKS})

add_executable(demo_thread thread.cpp)
