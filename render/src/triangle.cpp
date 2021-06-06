#include "../triangle.h"
#include <Eigen/Eigen>
#include <spdlog/spdlog.h>
#include "../utils.h"

Intersection Triangle::intersect(const std::shared_ptr<Triangle> &obj, const Ray &ray) {

    // 使用 Moller Trumbore 算法来计算
    auto e1 = obj->B() - obj->A();
    auto e2 = obj->C() - obj->A();
    auto s = ray.origin() - obj->A();
    auto s1 = ray.direction().get().cross(e2);
    auto s2 = s.cross(e1);

    float s1_dot_e1 = s1.dot(e1);
    if (std::abs(s1_dot_e1) <= std::numeric_limits<float>::epsilon())
        return Intersection::no_intersect();

    float t_near = s2.dot(e2) / s1_dot_e1;
    float b1 = s1.dot(s) / s1_dot_e1;
    float b2 = s2.dot(ray.direction().get()) / s1_dot_e1;

    // todo 在比较 t_near 时是否可以 epsilon，防止在自身弹射
    if (t_near > 0.f && b1 >= 0.f && b2 >= 0.f &&
        (1.f - b1 - b2) >= -std::numeric_limits<float>::min()) {
        return Intersection(ray.origin() + t_near * ray.direction().get(), obj->normal(), t_near, obj);
    }

    return Intersection::no_intersect();
}


Intersection Triangle::obj_sample(const std::shared_ptr<Triangle> &obj, float area_threshold) {
    assert(area_threshold - obj->area() < epsilon_7);

    // 在三角形内均匀地采样
    float x = std::sqrt(random_float_get());
    float y = random_float_get();

    auto inter_pos = obj->_A * (1.f - x) + obj->_B * (x * (1.f - y)) + obj->_C * (x * y);
    return Intersection(inter_pos, obj->_normal, -1.f, obj);
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
