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
    Triangle(Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2, std::shared_ptr<Material> mat)
            : _A(std::move(v0)),
              _B(std::move(v1)),
              _C(std::move(v2)) {

        this->_material = std::move(mat);
        this->init();
    }

    // 根据 Assimp 的顶点来初始化三角形
    Triangle(const aiVector3D &v0, const aiVector3D &v1, const aiVector3D &v2, std::shared_ptr<Material> mat) {
        _A = Eigen::Vector3f{v0.x, v0.y, v0.z};
        _B = Eigen::Vector3f{v1.x, v1.y, v1.z};
        _C = Eigen::Vector3f{v2.x, v2.y, v2.z};

        this->_material = std::move(mat);
        this->init();
    }

    // 从两个构造函数里面提取出来的公共部分
    void init() {
        // 计算面法线，逆时针方向向外
        this->_normal = (_B - _A).cross(_C - _B);

        // 计算包围盒
        this->_bounding_box = BoundingBox(_A, _B);
        this->_bounding_box.union_(_C);

        // 计算面积
        this->_area = (_B - _A).cross(_C - _A).norm() * 0.5f;
    }


    // =========================================================
    // 属性
    // =========================================================
    inline Eigen::Vector3f A() const {
        return this->_A;
    }

    inline Eigen::Vector3f B() const {
        return this->_B;
    }

    inline Eigen::Vector3f C() const {
        return this->_C;
    }

    inline Direction normal() const {
        return this->_normal;
    }

    inline ObjectType type_get() const override {
        return ObjectType::Triangle;
    }


    // =========================================================
    // 计算交点
    // =========================================================
    static Intersection intersect(const std::shared_ptr<Triangle> &obj, const Ray &ray);


    // =========================================================
    // 按物体采样
    // =========================================================
    static Intersection obj_sample(const std::shared_ptr<Triangle> &obj, float area_threshold);


private:
    Eigen::Vector3f _A, _B, _C;
    Direction _normal;
};


class MeshTriangle : public Object {
public:

    MeshTriangle(const std::shared_ptr<Material> &mat, const std::shared_ptr<BVH> root)
            : Object(root->bounding_box(), root->area(), mat),
              bvh(root) {}

    // =========================================================
    // 属性
    // =========================================================
    inline ObjectType type_get() const override {
        return ObjectType::MeshTriangle;
    }


    // =========================================================
    // 按物体采样
    // =========================================================
    static inline Intersection obj_sample(const std::shared_ptr<MeshTriangle> &obj, float area_threshold) {
        assert(area_threshold <= obj->_area);
        return obj->bvh->sample_obj(area_threshold);
    }


    // =========================================================
    // 计算交点
    // =========================================================
    static inline Intersection intersec(const std::shared_ptr<MeshTriangle> &obj, const Ray &ray) {
        return obj->bvh->intersect(ray);
    }


    // =========================================================
    // 根据 Assimp 生成模型
    // =========================================================
    // 从模型文件中载入模型，一个文件有多个 node，一个 node 有多个 mesh，树形结果
    static std::vector<std::shared_ptr<MeshTriangle>> mesh_load(const std::string &file_name);

    static std::vector<std::shared_ptr<MeshTriangle>> process_ainode(const aiNode &node, const aiScene &scene);

    static std::shared_ptr<MeshTriangle> process_aimesh(const aiMesh &mesh);


private:

    std::shared_ptr<BVH> bvh;
};


#endif //RENDER_DEBUG_TRIANGLE_H
