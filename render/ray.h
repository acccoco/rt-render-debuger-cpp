#ifndef RENDER_DEBUG_RAY_H
#define RENDER_DEBUG_RAY_H


#include <Eigen/Eigen>


// 方向类，为了确保所有的方向都是单位向量
class Direction {
public:
    Direction() : _vec{0.f, 0.f, 0.f} {}

    Direction(const Eigen::Vector3f &vec) : _vec(vec.normalized()) {}

    static inline Direction zero() {
        return Direction();
    }

    inline const Eigen::Vector3f &get() const {
        return this->_vec;
    }

    // 重载负号
    inline const Direction operator-() const {
        auto res = Direction();
        res._vec = -this->_vec;
        return res;
    }

private:

    Eigen::Vector3f _vec;
};


class Ray {
public:
    Ray(const Eigen::Vector3f &orig, const Eigen::Vector3f &dir)
            : _origin(orig), _direction(dir) {}

    Ray(const Eigen::Vector3f &orig, const Direction &dir)
            : _origin(orig), _direction(dir) {}

    // =========================================================
    // 属性
    // =========================================================
    inline Eigen::Vector3f origin() const { return this->_origin; }

    inline Direction direction() const { return this->_direction; }


private:
    Eigen::Vector3f _origin;
    Direction _direction;
};


#endif //RENDER_DEBUG_RAY_H
