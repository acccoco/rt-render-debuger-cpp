#ifndef RENDER_DEBUG_BVH_H
#define RENDER_DEBUG_BVH_H

#include <memory>
#include <utility>

#include "object.h"


/**
 * BVH 的树节点
 * 叶子节点一定有 object
 * 非叶子节点没有 object，一定有两个子节点
 */
class BVH {
public:
    /* 根据 obj 的列表构建 BVH 加速结构 */
    static std::shared_ptr<BVH> build(const std::vector<std::shared_ptr<Object>> &objs);

    /* 构造函数：创建非叶子节点 */
    BVH(BoundingBox box, float area_, std::shared_ptr<BVH> lchild_, std::shared_ptr<BVH> rchild_)
            : _box(std::move(box)),
              _area(area_),
              _lchild(std::move(lchild_)),
              _rchild(std::move(rchild_)),
              _object(nullptr) {}

    /* 构造函数：创建叶子节点 */
    BVH(BoundingBox box, float area_, std::shared_ptr<Object> obj)
            : _box(std::move(box)),
              _area(area_),
              _lchild(nullptr),
              _rchild(nullptr),
              _object(std::move(obj)) {}

    /* 计算 BVH 内的物体和光线的交点 */
    [[nodiscard]] Intersection intersect(const Ray &ray) const;

    /* 按照面积在 BVH 中随机的采样 */
    Intersection sample_obj(float area_threshold);


private:
    BoundingBox _box;                   /* 以当前节点为树根，BVH 树的包围盒 */
    float _area;                        /* BVH 中所有对象的表面积 */
    std::shared_ptr<BVH> _lchild;
    std::shared_ptr<BVH> _rchild;
    std::shared_ptr<Object> _object;    /* 当前节点包含的对象，只有叶子节点才有 */

public:
    // 属性


    [[nodiscard]] inline const std::shared_ptr<BVH> &lchild() const { return _lchild; }

    [[nodiscard]] inline const std::shared_ptr<BVH> &rchild() const { return _rchild; }

    [[nodiscard]] inline const std::shared_ptr<Object> &object() const { return _object; }

    [[nodiscard]] inline const BoundingBox &bounding_box() const { return _box; }

    [[nodiscard]] inline const float &area() const { return _area; }
};


/**
 * 将图形重心按照某个方向的坐标升序排序，找到第 k 大的
 * @param objs
 * @param k 第 k 大，从 0 开始
 * @param dir 重心按照哪个位置分量进行排序
 * @return [比 k 小的，第 k 大的，比 k 大的]
 */
std::tuple<std::vector<std::shared_ptr<Object>>, std::shared_ptr<Object>, std::vector<std::shared_ptr<Object>>>
find_kth_obj(const std::vector<std::shared_ptr<Object>> &objs, unsigned int k, ExtensionDir dir);


#endif //RENDER_DEBUG_BVH_H
