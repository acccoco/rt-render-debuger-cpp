#ifndef RENDER_DEBUG_BOUNDING_BOX_H
#define RENDER_DEBUG_BOUNDING_BOX_H

#include <Eigen/Eigen>


enum class ExtensionDir {
    X, Y, Z
};


class BoundingBox {
public:
    BoundingBox();

    BoundingBox(const Eigen::Vector3f &p);

    BoundingBox(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);

    // 最大延伸方向
    ExtensionDir max_extension() const;

    // 重心
    Eigen::Vector3f center() const;

    // 对角线
    Eigen::Vector3f diagonal() const;

    // 判断包围盒是否和射线相交
    bool
    is_intersect(const Eigen::Vector3f &origin, const Eigen::Vector3f &dir);

    // 两个包围盒做并集操作
    static BoundingBox union_(const BoundingBox &box1, const BoundingBox &box2);

    // 包含另一个点
    void union_(const Eigen::Vector3f &p);

    friend std::ostream &operator<<(std::ostream &os, const BoundingBox &box);


public:
    Eigen::Vector3f p_min, p_max;
};

#endif //RENDER_DEBUG_BOUNDING_BOX_H
