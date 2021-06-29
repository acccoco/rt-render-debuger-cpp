#include "scene.h"

#include <fmt/format.h>

#include "utils.h"


std::tuple<float, Intersection> Scene::sample_light() const {

    // 随机选择一个面积值
    float area_threshold = random_float_get() * _emit.total_area;

    // 按照固定的方法遍历所有的发光体，看面积值落在哪个发光体内
    // todo 遍历方法可以优化，时间复杂度从 n 到 logn
    float temp_total_area = 0.f;
    float area_threshold_obj;
    std::shared_ptr<Object> emit_obj;
    for (auto &obj : _emit.objs) {
        temp_total_area += obj->area();
        if (temp_total_area > area_threshold - epsilon_3) {
            emit_obj = obj;
            area_threshold_obj = area_threshold - (temp_total_area - obj->area());
        }
    }

    // 如果有发光体，就一定能找到
    // todo 这里出过问题，暂时的解决办法是增大 epsilon，如果再出，就详细排查
    //  排查方法可以是：在此处增加 if 判断
    assert(_emit.objs.empty() ^ (emit_obj != nullptr));

    // 如果没有找到
    if (!emit_obj) {
        return {0.f, Intersection::no_intersect()};
    }

    // 如果找到了：去物体内部采样
    return {
            1.f / emit_obj->area(),
            emit_obj->obj_sample(area_threshold_obj)
            // Object::sample_obj(emit_obj, area_threshold_obj)
    };
}

void Scene::obj_add(const std::shared_ptr<Object> &obj) {
    if (!obj) return;

    this->_objs.push_back(obj);

    if (obj->mat()->is_emission()) {
        this->_emit.objs.push_back(obj);
        this->_emit.total_area += obj->area();
    }
}


void Scene::initInverseViewMatrix() {
    // 防止死锁
    assert(std::abs(this->_camera.look_at.get().y()) < 0.9f);

    Eigen::Vector3f right = this->_camera.look_at.get().cross(Eigen::Vector3f(0.f, 1.f, 0.f));
    Eigen::Vector3f up = right.cross(this->_camera.look_at.get());

    auto &i = right;
    auto &j = up;
    auto k = -this->_camera.look_at.get();

    this->_camera.view_matrix_inverse << i.x(), j.x(), k.x(), this->_camera.pos.x(),
            i.y(), j.y(), k.y(), this->_camera.pos.y(),
            i.z(), j.z(), k.z(), this->_camera.pos.z(),
            0, 0, 0, 1;
}
