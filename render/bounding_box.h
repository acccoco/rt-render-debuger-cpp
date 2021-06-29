#ifndef RENDER_DEBUG_BOUNDING_BOX_H
#define RENDER_DEBUG_BOUNDING_BOX_H

#include <Eigen/Eigen>

#include "ray.h"


/**
 * AABB 的最大延伸方向
 * AABB 的三个变长分别平行于 x，y，z 轴
 * 最大延伸方向指：变长最长的的那个轴向
 */
enum class ExtensionDir {
    X, Y, Z
};


/**
 * AABB 包围盒
 */
class BoundingBox {
public:
    /**
     * 默认创建的包围盒：
     * - maxP = (+max, +max, +max)
     * - minP = (-max, -max, -max)
     */
    BoundingBox();

    /* 通过空间中一个点来创建包围盒，使得包围盒只包含了这一个点 */
    explicit BoundingBox(const Eigen::Vector3f &p)
            : p_min(p), p_max(p) {}

    /* 通过空间中的两个点来创建一个包围盒，使得包围盒恰好包含了这两个点 */
    BoundingBox(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);

    /* 最大延伸方向：最大的变长在那个方向上 */
    [[nodiscard]] ExtensionDir maxExtension() const;

    /* 包围盒的体积中心 */
    [[nodiscard]] inline Eigen::Vector3f center() const {
        return 0.5f * p_min + 0.5f * p_max;
    }

    /* 包围盒的对角线对角线 */
    [[nodiscard]] inline Eigen::Vector3f diagonal() const {
        return p_max - p_min;
    }

    /* 判断包围盒是否和射线相交 */
    [[nodiscard]] bool isIntersect(const Ray &ray) const;

    /* 包围盒是否包含某个点 */
    [[nodiscard]] bool contain(const Eigen::Vector3f &point) const;

    /* 两个包围盒做并集操作：得到一个新的包围盒*/
    static BoundingBox unionOp(const BoundingBox &box1, const BoundingBox &box2);

    /* 与空间中一个点做并运算：使当前包围盒包含空间中的点 */
    void unionOp(const Eigen::Vector3f &p);

    /* 与另一个包围盒做并运算：使当前包围盒包含另一个包围盒 */
    void unionOp(const BoundingBox &box);

    /* 支持 std::cout << AABB 的方式来显示包围盒的信息 */
    friend std::ostream &operator<<(std::ostream &os, const BoundingBox &box);


public:
    /* 包围盒三个方向最小的点和最大的点；通过这两个点，可以定义一个包围盒 */
    Eigen::Vector3f p_min, p_max;
};

#endif //RENDER_DEBUG_BOUNDING_BOX_H
