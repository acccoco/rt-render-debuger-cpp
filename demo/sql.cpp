//
// Created by 漆正杰 on 2021/6/1.
//
#include <iostream>
#include <stdexcept>
#include <sqlite3.h>

#ifndef DB_PATH
#error no database path defined
#endif


const int BUFFER_SIZE = 512;
char buffer[BUFFER_SIZE];
char *err_msg;


int main() {
    /* open database */
    sqlite3 *db = nullptr;
    if (SQLITE_OK != sqlite3_open(DB_PATH, &db)) {
        throw std::runtime_error("error");
    }

    /* insert */
    snprintf(buffer, BUFFER_SIZE, "insert into user(name, age) values('%s', '%d')", "aoe", 12);
    if (SQLITE_OK != sqlite3_exec(db, buffer, nullptr, nullptr, &err_msg)) {
        throw std::runtime_error(err_msg);
    }

    /* select */
    snprintf(buffer, BUFFER_SIZE, "select name from user");
    sqlite3_stmt *stmt = nullptr;
    if (SQLITE_OK != sqlite3_prepare_v2(db, buffer, -1, &stmt, nullptr)) {
        throw std::runtime_error("fail to select");
    }
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        auto name = sqlite3_column_text(stmt, 0);
        std::cout << "name is: " << name << std::endl;
    }

    /* close database */
    sqlite3_close(db);
    return 0;
}

