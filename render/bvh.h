#ifndef RENDER_DEBUG_BVH_H
#define RENDER_DEBUG_BVH_H


#include <memory>

#include "object.h"


// BVH 的树节点
// 只有子节点才会有 object，非叶子节点一定有两个子节点
class BVH {
public:

    BVH(const BoundingBox &box, float area_, const std::shared_ptr<BVH> &lchild_, const std::shared_ptr<BVH> &rchild_)
            : _box(box),
              _area(area_),
              _lchild(lchild_),
              _rchild(rchild_),
              _object(nullptr) {}

    BVH(const BoundingBox &box, float area_, const std::shared_ptr<Object> &obj)
            : _box(box),
              _area(area_),
              _lchild(nullptr),
              _rchild(nullptr),
              _object(obj) {}

    // =========================================================
    // 递归构造 BVH
    // =========================================================
    static std::shared_ptr<BVH> build(const std::vector<std::shared_ptr<Object>> &objs);


    // =========================================================
    // 属性
    // =========================================================
    inline const std::shared_ptr<BVH> &lchild() const {
        return _lchild;
    }

    inline const std::shared_ptr<BVH> &rchild() const {
        return _rchild;
    }

    inline const std::shared_ptr<Object> &object() const {
        return _object;
    }

    inline const BoundingBox &bounding_box() const {
        return _box;
    }

    inline const float &area() const {
        return _area;
    }

    // =========================================================
    // 计算交点
    // =========================================================
    Intersection intersect(const Ray &ray) const;


    // =========================================================
    // 按物体采样
    // =========================================================
    Intersection sample_obj(float area_threshold);


private:
    BoundingBox _box;
    float _area;
    std::shared_ptr<BVH> _lchild;
    std::shared_ptr<BVH> _rchild;
    std::shared_ptr<Object> _object;
};


// 将图形重心按照某个方向的坐标升序排序，找到第 k 大的
std::tuple<std::vector<std::shared_ptr<Object>>, std::shared_ptr<Object>, std::vector<std::shared_ptr<Object>>>
find_kth_obj(const std::vector<std::shared_ptr<Object>> &objs, unsigned int k, ExtensionDir dir);


#endif //RENDER_DEBUG_BVH_H
