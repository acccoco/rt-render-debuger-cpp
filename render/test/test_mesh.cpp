#define CATCH_CONFIG_MAIN

#include <string>
#include <fstream>

#include <fmt/format.h>
#include <catch2/catch.hpp>

#include "config.h"
#include "../utils.h"
#include "../triangle.h"


// =========================================================
// 测试基于 Assimp 的模型导入功能是否正常
// =========================================================
TEST_CASE("load obj file")
{
    SECTION("cornellbox")
    {

        std::vector<std::string> file_path_list;
        for (auto file : {PATH_CORNELL_FLOOR,
                          PATH_CORNELL_LEFT,
                          PATH_CORNELL_LIGHT,
                          PATH_CORNELL_RIGHT,
                          PATH_CORNELL_SHORTBOX,
                          PATH_CORNELL_TALLBOX}) {
            file_path_list.emplace_back(file);
        }

        SECTION("确保文件存在")
        {
            std::fstream fs;
            for (auto &file_path : file_path_list) {
                fs.open(file_path, std::ios::in);
                REQUIRE(fs);
                fs.close();
            }
        }

        SECTION("open with Assimp")
        {
            for (auto &file_path : file_path_list) {
                auto meshes = MeshTriangle::mesh_load(file_path);
                REQUIRE(!meshes.empty());
                REQUIRE(meshes[0]->area() > 0);
            }
        }
    }
}
