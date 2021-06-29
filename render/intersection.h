#ifndef RENDER_DEBUG_INTERSECTION_H
#define RENDER_DEBUG_INTERSECTION_H

#include <memory>
#include <utility>

#include <Eigen/Eigen>

#include "ray.h"
#include "material.h"


/* 射线和物体发生相交，交点的信息 */
class Intersection {
public:

    /* 工厂函数：获得没有相交的 Intersection 对象 */
    static inline Intersection no_intersect() {
        return Intersection();
    }

    /* 构造函数：没有相交的情况 */
    Intersection()
            : _happened(false),
              _position(0.f, 0.f, 0.f),
              _normal(),
              _t_near(-1.f),
              _mat(nullptr) {}

    /* 构造函数：有相交的情况 */
    Intersection(Eigen::Vector3f position, Direction normal, float t_near_, std::shared_ptr<Material> mat)
            : _happened(true),
              _position(std::move(position)),
              _normal(std::move(normal)),
              _t_near(t_near_),
              _mat(std::move(mat)) {}


private:
    bool _happened;                         /* 是否发生了相交 */
    Eigen::Vector3f _position;              /* 交点的坐标 */
    Direction _normal;                      /* 交点的法线 */
    float _t_near;                          /* 光线起点到交点的距离 */
    std::shared_ptr<Material> _mat;         /* 发生相交时，物体的材质 */


public:
    // 属性

    [[nodiscard]] inline bool happened() const { return this->_happened; }

    [[nodiscard]] inline const Eigen::Vector3f &pos() const { return this->_position; }

    [[nodiscard]] inline const Direction &normal() const { return this->_normal; }

    [[nodiscard]] inline float t_near() const { return this->_t_near; };

    [[nodiscard]] inline std::shared_ptr<Material> mat() const { return _mat; }

};

#endif //RENDER_DEBUG_INTERSECTION_H
