#ifndef RENDER_DEBUG_RAY_H
#define RENDER_DEBUG_RAY_H

#include <utility>

#include <Eigen/Eigen>


/**
 * 表示方向
 * 通过静态类型来保证方向的大小是 1，不用再手动 normalize
 */
class Direction {
public:

    /* 工厂函数：返回所有分量都是 0 的方向对象 */
    static inline Direction zero() { return Direction(); }

    Direction() : _vec{0.f, 0.f, 0.f} {}

    explicit Direction(const Eigen::Vector3f &vec) : _vec(vec.normalized()) {}

    [[nodiscard]] inline const Eigen::Vector3f &get() const { return _vec; }

    /* 支持负号运算 */
    inline Direction operator-() const {
        auto res = Direction();
        res._vec = -this->_vec;
        return res;
    }

private:

    Eigen::Vector3f _vec;           /* 用于表示方向的向量 */
};


/**
 * 表示射线
 * 射线由原点和方向组成
 */
class Ray {
public:
    Ray(Eigen::Vector3f orig, const Eigen::Vector3f& dir)
            : _origin(std::move(orig)), _direction(dir) {}

    Ray(Eigen::Vector3f orig, Direction dir)
            : _origin(std::move(orig)), _direction(std::move(dir)) {}


private:
    Eigen::Vector3f _origin;            /* 光线的原点 */
    Direction _direction;               /* 光线的方向 */

public:
    [[nodiscard]] inline Eigen::Vector3f origin() const { return this->_origin; }

    [[nodiscard]] inline Direction direction() const { return this->_direction; }
};


#endif //RENDER_DEBUG_RAY_H
