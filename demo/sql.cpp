//
// Created by 漆正杰 on 2021/6/1.
//
#include <iostream>
#include <stdexcept>
#include <sqlite3.h>
#include <fmt/format.h>
#include "config.h"
#include <chrono>
#include <thread>


int main() {
    char *err_msg;

    /* open database */
    sqlite3 *db = nullptr;
    if (SQLITE_OK != sqlite3_open(DB_PATH, &db)) {
        throw std::runtime_error("error");
    }

    /* insert */
    auto insert_str = fmt::format("insert into user(name, age) values('{}', '{}')", "ao", 12);
    if (SQLITE_OK != sqlite3_exec(db, insert_str.c_str(), nullptr, nullptr, &err_msg)) {
        throw std::runtime_error(err_msg);
    }

    /* select */
    auto select_str = fmt::format("select name from user");
    sqlite3_stmt *stmt = nullptr;
    if (SQLITE_OK != sqlite3_prepare_v2(db, select_str.c_str(), -1, &stmt, nullptr)) {
        throw std::runtime_error("fail to select");
    }
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        auto name = sqlite3_column_text(stmt, 0);
        std::cout << "name is: " << name << std::endl;
    }

    for (int i = 0; i < 10; ++i) {
        fmt::print("\rprogress: {}%.", i);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    /* close database */
    sqlite3_close(db);
    return 0;
}

