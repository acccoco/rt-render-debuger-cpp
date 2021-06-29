#include "triangle.h"

#include <Eigen/Eigen>
#include <spdlog/spdlog.h>

#include "utils.h"


/**
 * 使用 Moller Trumbore 算法来计算光线和三角形的交点
 *  b1 和 b2 表示三角形重心差值参数
 * @param ray
 * @return
 */
Intersection Triangle::intersect(const Ray &ray) {
    // Moller Trumbore 算法
    auto E1 = this->B() - this->A();
    auto E2 = this->C() - this->A();
    auto S = ray.origin() - this->A();
    auto S1 = ray.direction().get().cross(E2);
    auto S2 = S.cross(E1);

    float S1_dot_S2 = S1.dot(E1);
    if (std::abs(S1_dot_S2) <= std::numeric_limits<float>::epsilon())
        return Intersection::no_intersect();

    float t_near = S2.dot(E2) / S1_dot_S2;
    float b1 = S1.dot(S) / S1_dot_S2;
    float b2 = S2.dot(ray.direction().get()) / S1_dot_S2;

    // todo 在比较 t_near 时是否可以 epsilon，防止在自身弹射
    if (t_near > 0.f && b1 >= 0.f && b2 >= 0.f &&
        (1.f - b1 - b2) >= -std::numeric_limits<float>::min()) {
        return Intersection(ray.origin() + t_near * ray.direction().get(),
                            this->normal(),
                            t_near,
                            this->mat());
    }

    return Intersection::no_intersect();
}

Intersection Triangle::obj_sample(float area_threshold) {
    assert(area_threshold - this->area() < epsilon_7);

    // 在三角形内均匀地采样
    float x = std::sqrt(random_float_get());
    float y = random_float_get();

    auto inter_pos = this->_a * (1.f - x) + this->_b * (x * (1.f - y)) + this->_c * (x * y);
    return Intersection(inter_pos, this->_normal, -1.f, this->mat());
}


std::vector<std::shared_ptr<MeshTriangle>> MeshTriangle::mesh_load(const std::string &file_path) {

    SPDLOG_INFO("try to load scene from file: {}", file_path);

    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(file_path, aiProcess_Triangulate | aiProcess_FlipUVs);
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        SPDLOG_ERROR("fail to load scene, error message: {}", importer.GetErrorString());
        return {};
    }

    return process_ainode(*scene->mRootNode, *scene);
}


std::shared_ptr<MeshTriangle> MeshTriangle::process_aimesh(const aiMesh &mesh) {

    std::vector<std::shared_ptr<Object>> objs(mesh.mNumFaces);

    // todo 这里使用了默认的灰色材质
    auto mat = Material::diffuse_mat();

    SPDLOG_INFO("mesh triangle num: {}", mesh.mNumFaces);
    for (unsigned int i = 0; i < mesh.mNumFaces; ++i) {
        const aiFace &face = mesh.mFaces[i];

        // 根据 Assimp Mesh 的 face 生成三角形
        auto triangle = std::make_shared<Triangle>(mesh.mVertices[face.mIndices[0]],
                                                   mesh.mVertices[face.mIndices[1]],
                                                   mesh.mVertices[face.mIndices[2]],
                                                   mat);
        objs[i] = triangle;
    }

    // 构建 BVH
    auto bvh_root = BVH::build(objs);

    return std::make_shared<MeshTriangle>(mat, bvh_root);
}


std::vector<std::shared_ptr<MeshTriangle>>
MeshTriangle::process_ainode(const aiNode &node, const aiScene &scene) {

    std::vector<std::shared_ptr<MeshTriangle>> meshes;

    // 处理当前节点
    for (unsigned int i = 0; i < node.mNumMeshes; ++i) {
        unsigned int mesh_id = node.mMeshes[i];
        meshes.push_back(process_aimesh(*scene.mMeshes[mesh_id]));
    }

    // 处理子节点
    for (unsigned int i = 0; i < node.mNumChildren; ++i) {
        auto child_meshes = process_ainode(*node.mChildren[i], scene);
        meshes.insert(meshes.end(), child_meshes.begin(), child_meshes.end());
    }

    return meshes;
}

