#include "../triangle.h"

#include <utility>


Triangle::Triangle(Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2)
        : A(std::move(v0)), B(std::move(v1)), C(std::move(v2)) {
    // 计算面法线，逆时针方向向外
    this->face_normal = (B - A).cross(C - B).normalized();
}


