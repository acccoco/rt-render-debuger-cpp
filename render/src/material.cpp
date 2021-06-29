#include "material.h"
#include "utils.h"


/**
 * 按照立体角，在半球内随机均匀地采样
 *  - z 坐标是均匀分布的
 *  - phi 是均匀分布的
 */
std::tuple<float, Direction> Material::sample_himsphere_random(const Direction &N) {

    // 半球采样，pdf 是半球的立体角（2pi）分之一
    float z = random_float_get();
    float phi = random_float_get() * (float) M_PI * 2.f;

    float r = std::sqrt(1 - z * z);
    Direction local({r * std::cos(phi), r * std::sin(phi), z});
    return {0.5f / M_PI, local_to_world(N, local)};
}


/**
 * 将局部坐标系中的向量转换为全局坐标系的向量
 * - 以固定的方式，通过物体表面法线来建立局部坐标系
 * - 通过坐标系变换，将局部的向量变换为全局的向量
 */
Direction Material::local_to_world(const Direction &N, const Direction &local) {
    Eigen::Vector3f B, C;

    // 如果 N 的两个分量都为 0，需要确保新得到的 C 向量不是 0 向量
    if (std::abs(N.get().x()) > std::abs(N.get().y())) {
        C = {N.get().z(), 0.f, -N.get().x()};
    } else {
        C = {0.f, N.get().z(), -N.get().y()};
    }
    C.normalize();
    B = C.cross(N.get());

    return Direction{local.get().x() * B + local.get().y() * C + local.get().z() * N.get()};
}
