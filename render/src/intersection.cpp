#include "../intersection.h"

#include "../object.h"
#include "../triangle.h"


Intersection Intersection::intersect(const std::shared_ptr<Object> &obj, const Ray &ray) {
    switch (obj->type_get()) {

        case ObjectType::Triangle:
            return Triangle::intersect(std::dynamic_pointer_cast<Triangle>(obj), ray);
        case ObjectType::MeshTriangle:
            return MeshTriangle::intersec(std::dynamic_pointer_cast<MeshTriangle>(obj), ray);
        default:
            throw std::runtime_error("never");

    }
}

