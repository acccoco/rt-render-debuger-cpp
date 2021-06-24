#include "bounding_box.h"
#include "utils.h"


ExtensionDir BoundingBox::max_extension() const {
    auto d = diagonal();
    if (d.x() > d.y() && d.x() > d.z()) return ExtensionDir::X;
    else if (d.y() > d.z()) return ExtensionDir::Y;
    else return ExtensionDir::Z;
}

Eigen::Vector3f BoundingBox::diagonal() const {
    return p_max - p_min;
}

Eigen::Vector3f BoundingBox::center() const {
    return 0.5f * p_min + 0.5f * p_max;
}

inline bool intersect_partial(float &t_min, float &t_max,
                              const float origin, const float direction,
                              const float box_min, const float box_max) {
    // 射线与包围盒平行
    if (std::abs(direction) < std::numeric_limits<float>::epsilon()) {
        if (origin > box_max || origin < box_min)
            return false;
        t_min = -std::numeric_limits<float>::infinity();
        t_max = std::numeric_limits<float>::infinity();
        return true;
    }

    // 射线不与包围盒平行
    t_min = (box_min - origin) / direction;
    t_max = (box_max - origin) / direction;
    if (t_min > t_max) std::swap(t_min, t_max);
    return t_max > 0.f;
}

bool BoundingBox::is_intersect(const Ray &ray) const {
    float t_min_x, t_max_x, t_min_y, t_max_y, t_min_z, t_max_z;
    if (!intersect_partial(t_min_x, t_max_x, ray.origin().x(), ray.direction().get().x(), p_min.x(), p_max.x()))
        return false;
    if (!intersect_partial(t_min_y, t_max_y, ray.origin().y(), ray.direction().get().y(), p_min.y(), p_max.y()))
        return false;
    if (!intersect_partial(t_min_z, t_max_z, ray.origin().z(), ray.direction().get().z(), p_min.z(), p_max.z()))
        return false;

    float t_min = std::max(t_min_x, std::max(t_min_y, t_min_z));
    float t_max = std::min(t_max_x, std::min(t_max_y, t_max_z));
    return t_min <= t_max && t_max > 0;
}


// 找到两个点所有维度的最大值，生成一个新的点
inline Eigen::Vector3f max_p(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) {
    return Eigen::Vector3f{
            std::max(p1.x(), p2.x()),
            std::max(p1.y(), p2.y()),
            std::max(p1.z(), p2.z())
    };
}

// 找到两个点所有维度的最小值，生成一个新的点
inline Eigen::Vector3f min_p(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) {
    return Eigen::Vector3f{
            std::min(p1.x(), p2.x()),
            std::min(p1.y(), p2.y()),
            std::min(p1.z(), p2.z())
    };
}

BoundingBox BoundingBox::union_(const BoundingBox &box1, const BoundingBox &box2) {
    BoundingBox box;
    box.p_max = max_p(box1.p_max, box2.p_max);
    box.p_min = min_p(box1.p_min, box2.p_min);
    return box;
}

BoundingBox::BoundingBox() {
    float min_num = std::numeric_limits<float>::lowest();
    float max_num = std::numeric_limits<float>::max();

    p_min = Eigen::Vector3f{max_num, max_num, max_num};
    p_max = Eigen::Vector3f{min_num, min_num, min_num};
}


void BoundingBox::union_(const Eigen::Vector3f &p) {
    p_max = max_p(p_max, p);
    p_min = min_p(p_min, p);
}

std::ostream &operator<<(std::ostream &os, const BoundingBox &box) {
    os << "p_min: " << box.p_min.transpose() << std::endl;
    os << "p_max: " << box.p_max.transpose() << std::endl;
    return os;
}

BoundingBox::BoundingBox(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) {
    p_max = max_p(p1, p2);
    p_min = min_p(p1, p2);
}

void BoundingBox::union_(const BoundingBox &box) {
    p_max = max_p(p_max, box.p_max);
    p_min = min_p(p_min, box.p_min);
}

bool BoundingBox::contain(const Eigen::Vector3f &point) const {

    if (point.x() < p_min.x() - epsilon_4 || point.x() > p_max.x() + epsilon_4) return false;
    if (point.y() < p_min.y() - epsilon_4 || point.y() > p_max.y() + epsilon_4) return false;
    if (point.z() < p_min.z() - epsilon_4 || point.z() > p_max.z() + epsilon_4) return false;
    return true;
}
