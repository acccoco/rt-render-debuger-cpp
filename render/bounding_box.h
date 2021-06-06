#ifndef RENDER_DEBUG_BOUNDING_BOX_H
#define RENDER_DEBUG_BOUNDING_BOX_H

#include <Eigen/Eigen>

#include "ray.h"

enum class ExtensionDir {
    X, Y, Z
};


class BoundingBox {
public:
    BoundingBox();

    BoundingBox(const Eigen::Vector3f &p) : p_min(p), p_max(p) {}

    BoundingBox(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);

    // 最大延伸方向
    ExtensionDir max_extension() const;

    // 重心
    Eigen::Vector3f center() const;

    // 对角线
    Eigen::Vector3f diagonal() const;

    // 判断包围盒是否和射线相交
    bool is_intersect(const Ray &ray) const;

    // 包围盒是否包含某个点
    bool contain(const Eigen::Vector3f &point) const;

    // 两个包围盒做并集操作
    static BoundingBox union_(const BoundingBox &box1, const BoundingBox &box2);

    // 包含另一个点
    void union_(const Eigen::Vector3f &p);

    void union_(const BoundingBox &box);

    friend std::ostream &operator<<(std::ostream &os, const BoundingBox &box);


public:
    Eigen::Vector3f p_min, p_max;
};

#endif //RENDER_DEBUG_BOUNDING_BOX_H
