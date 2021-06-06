#ifndef RENDER_DEBUG_INTERSECTION_H
#define RENDER_DEBUG_INTERSECTION_H


#include <memory>

#include <Eigen/Eigen>

#include "ray.h"


class Object;

class Intersection {
public:
    // 没有相交的情况
    Intersection()
            : _happened(false),
              _position(0.f, 0.f, 0.f),
              _normal(),
              _t_near(-1.f),
              _obj(nullptr) {}

    // 没有相交
    static inline Intersection no_intersect() {
        return Intersection();
    }

    // 有相交的情况
    Intersection(Eigen::Vector3f position, Direction normal, float t_near_, std::shared_ptr<Object> obj)
            : _happened(true),
              _position(std::move(position)),
              _normal(std::move(normal)),
              _t_near(t_near_),
              _obj(std::move(obj)) {}


    // =========================================================
    // 属性
    // =========================================================
    inline bool happened() const { return this->_happened; }

    inline const Eigen::Vector3f &pos() const { return this->_position; }

    inline const Direction &normal() const { return this->_normal; }

    inline float t_near() const { return this->_t_near; };

    inline std::shared_ptr<Object> obj() const { return this->_obj; }


    // =========================================================
    // 判读是否相交
    // =========================================================
    static Intersection intersect(const std::shared_ptr<Object> &obj, const Ray &ray);


private:
    bool _happened;
    Eigen::Vector3f _position;
    Direction _normal;
    float _t_near;
    std::shared_ptr<Object> _obj;
};


#endif //RENDER_DEBUG_INTERSECTION_H
