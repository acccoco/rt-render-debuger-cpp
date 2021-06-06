#ifndef RENDER_DEBUG_OBJECT_H
#define RENDER_DEBUG_OBJECT_H


#include <stdexcept>
#include "ray.h"
#include "bounding_box.h"
#include "intersection.h"
#include "material.h"


enum class ObjectType {
    Null,
    MeshTriangle,
    Triangle,
};


class Object {
public:
    Object() : _bounding_box(), _area(0.f), _material(nullptr) {}

    Object(BoundingBox box, float area, std::shared_ptr<Material> mat)
            : _bounding_box(std::move(box)), _area(area), _material(std::move(mat)) {}

    virtual ~Object() {}

    // =========================================================
    // 属性
    // =========================================================
    inline virtual ObjectType type_get() const {
        return ObjectType::Null;
    }

    inline virtual BoundingBox bounding_box() {
        return this->_bounding_box;
    }

    inline virtual const Material material() const {
        return *this->_material;
    }

    inline virtual Material &material() {
        return *this->_material;
    }

    inline virtual const float area() const {
        return this->_area;
    }


    // =========================================================
    // 采样物体
    // =========================================================
    static Intersection sample_obj(const std::shared_ptr<Object> &obj, float area_threshold);


protected:

    BoundingBox _bounding_box;
    float _area;
    std::shared_ptr<Material> _material;
};


#endif //RENDER_DEBUG_OBJECT_H
