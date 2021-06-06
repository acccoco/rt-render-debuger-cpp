#include "../object.h"
#include "../triangle.h"

Intersection Object::sample_obj(const std::shared_ptr<Object> &obj, float area_threshold) {
    switch (obj->type_get()) {
        case ObjectType::Triangle:
            return Triangle::obj_sample(std::dynamic_pointer_cast<Triangle>(obj), area_threshold);
        case ObjectType::MeshTriangle:
            return MeshTriangle::obj_sample(std::dynamic_pointer_cast<MeshTriangle>(obj), area_threshold);
        default:
            throw std::runtime_error("never");
    }
}
