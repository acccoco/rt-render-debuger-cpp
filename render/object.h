#ifndef RENDER_DEBUG_OBJECT_H
#define RENDER_DEBUG_OBJECT_H

#include <stdexcept>

#include "ray.h"
#include "material.h"
#include "bounding_box.h"
#include "intersection.h"


class Object {
public:
    Object()
            : _bounding_box(), _area(0.f), _material(nullptr) {}

    Object(BoundingBox box, float area, std::shared_ptr<Material> mat)
            : _bounding_box(std::move(box)), _area(area), _material(std::move(mat)) {}

    virtual ~Object() = default;

    /* 计算光线和当前物体的交点 */
    virtual Intersection intersect(const Ray &ray) = 0;

    /**
     * 对物体随机采样
     * @param area_threshold 面积阈值的参考值
     */
    virtual Intersection obj_sample(float area_threshold) = 0;

protected:
    BoundingBox _bounding_box;                  /* 物体的包围盒 */
    float _area;                                /* 物体的总面积 */
    std::shared_ptr<Material> _material;        /* 物体的材质 */

public:
    // 属性

    inline BoundingBox bounding_box() { return _bounding_box; }

    inline std::shared_ptr<Material> mat() { return _material; }

    [[nodiscard]] inline float area() const { return _area; }
};

#endif //RENDER_DEBUG_OBJECT_H
