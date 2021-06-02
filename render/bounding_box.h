#ifndef RENDER_DEBUG_BOUNDING_BOX_H
#define RENDER_DEBUG_BOUNDING_BOX_H

#include <Eigen/Eigen>


enum class ExtensionDir {
    X, Y, Z
};


class BoundingBox {
public:
    // 最大延伸方向
    ExtensionDir max_extension();

    // 重心
    Eigen::Vector3f center();

    // 判断包围盒是否和设想相交
    bool is_intersect(const Eigen::Vector3f &origin, const Eigen::Vector3f &dir);

    // 两个包围盒做并集操作
    static BoundingBox union_(const BoundingBox &box1, const BoundingBox &box2);


private:
    Eigen::Vector3f p_min, p_max;
};

#endif //RENDER_DEBUG_BOUNDING_BOX_H
