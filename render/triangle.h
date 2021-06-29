#ifndef RENDER_DEBUG_TRIANGLE_H
#define RENDER_DEBUG_TRIANGLE_H

#include <Eigen/Eigen>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include "bvh.h"
#include "object.h"
#include "intersection.h"


class Triangle : public Object {
public:
    /* 通过三个顶点来创建一个三角形，并指定三角形的材质 */
    Triangle(Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, std::shared_ptr<Material> mat)
            : _a(std::move(v0)),
              _b(std::move(v1)),
              _c(std::move(v2)) {

        this->_material = std::move(mat);
        this->init();
    }

    /* 根据 Assimp 的顶点来初始化三角形 */
    Triangle(const aiVector3D &v0, const aiVector3D &v1, const aiVector3D &v2, std::shared_ptr<Material> mat) {
        _a = Eigen::Vector3f{v0.x, v0.y, v0.z};
        _b = Eigen::Vector3f{v1.x, v1.y, v1.z};
        _c = Eigen::Vector3f{v2.x, v2.y, v2.z};

        this->_material = std::move(mat);
        this->init();
    }

    /* 初始化三角形，从两个构造函数中抽出的公共部分 */
    void init() {
        // 计算面法线，逆时针方向向外
        this->_normal = Direction((_b - _a).cross(_c - _b));

        // 计算包围盒
        this->_bounding_box = BoundingBox(_a, _b);
        this->_bounding_box.unionOp(_c);

        // 计算面积
        this->_area = (_b - _a).cross(_c - _a).norm() * 0.5f;
    }

    /* 计算三角形和光线的交点 */
    Intersection intersect(const Ray &ray) override;

    /* 在物体内随机采样 */
    Intersection obj_sample(float area_threshold) override;

private:
    Eigen::Vector3f _a, _b, _c;     /* 三角形三个顶点的坐标 */
    Direction _normal;              /* 三角形的面法线 */

public :
    [[nodiscard]] inline Eigen::Vector3f A() const { return _a; }

    [[nodiscard]] inline Eigen::Vector3f B() const { return _b; }

    [[nodiscard]] inline Eigen::Vector3f C() const { return _c; }

    [[nodiscard]] inline Direction normal() const { return _normal; }
};


class MeshTriangle : public Object {
public:
    // =========================================================
    // 根据 Assimp 生成模型
    // =========================================================
    // 从模型文件中载入模型，一个文件有多个 node，一个 node 有多个 mesh，树形结果
    static std::vector<std::shared_ptr<MeshTriangle>> mesh_load(const std::string &file_name);

    static std::vector<std::shared_ptr<MeshTriangle>> process_ainode(const aiNode &node, const aiScene &scene);

    static std::shared_ptr<MeshTriangle> process_aimesh(const aiMesh &mesh);


    /* 构造函数 */
    MeshTriangle(const std::shared_ptr<Material> &mat, const std::shared_ptr<BVH> &root)
            : Object(root->bounding_box(), root->area(), mat),
              bvh(root) {}

    /* 在模型内随机采样，area_threshold 是参考的面积阈值 */
    inline Intersection obj_sample(float area_threshold) override {
        assert(area_threshold <= this->_area);
        return this->bvh->sample_obj(area_threshold);
    }

    /* 计算模型和射线的交点 */
    inline Intersection intersect(const Ray &ray) override {
        return this->bvh->intersect(ray);
    }


private:
    std::shared_ptr<BVH> bvh;           /* 三角形模型由众多三角形组成，以 BVH 建立加速架构 */

};


#endif //RENDER_DEBUG_TRIANGLE_H
