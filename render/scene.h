#ifndef RENDER_DEBUG_SCENE_H
#define RENDER_DEBUG_SCENE_H

#include <memory>

#include <Eigen/Eigen>

#include "bvh.h"
#include "object.h"
#include "intersection.h"

class Scene {
public:

    Scene(int screen_width, int screen_height, float fov, const Eigen::Vector3f &camera_look_at, const Eigen::Vector3f &camera_pos)
            : _screen_width(screen_width), _screen_height(screen_height), _fov(fov), _camera{camera_look_at, camera_pos} {
        assert(screen_height > 0);
        assert(fov > 0.f && fov < 180.f);
        assert(camera_look_at.norm() > 0.f);
        assert(std::abs(camera_look_at.y()) < 0.9f);

        this->_gen_camera_matrix();
    }


    // =========================================================
    // 属性
    // =========================================================
    inline int screen_width() const { return _screen_width; }

    inline int screen_height() const { return _screen_height; }

    inline float fov() const { return _fov; }

    inline Eigen::Vector3f camera_pos() const { return _camera.pos; };

    // 从摄像机坐标系变换到 global 坐标系
    inline Eigen::Vector4f local_to_global(const Eigen::Vector4f &vec) {
        return this->_camera.local_to_global * vec;
    }

    // 建立加速结构
    void build() {
        this->_bvh = BVH::build(this->_objs);
    }

    void obj_add(const std::shared_ptr<Object> &obj);

    inline Intersection intersect(const Ray &ray) const {
        return this->_bvh->intersect(ray);
    }

    inline const auto &emit() const { return this->_emit; }

    // fixme 对场景中所有的光源进行采样，自己实现，可能会有问题
    std::tuple<float, Intersection> sample_light() const;

private:
    int _screen_width, _screen_height;
    float _fov;

    struct {
        Direction look_at;
        Eigen::Vector3f pos;
        Eigen::Matrix4f local_to_global;
    } _camera;

    // 生成摄像机的矩阵
    inline void _gen_camera_matrix() {
        // 防止死锁
        assert(std::abs(this->_camera.look_at.get().y()) < 0.9f);

        Eigen::Vector3f right = this->_camera.look_at.get().cross(Eigen::Vector3f(0.f, 1.f, 0.f));
        Eigen::Vector3f up = right.cross(this->_camera.look_at.get());

        auto &i = right;
        auto &j = up;
        auto k = -this->_camera.look_at.get();

        this->_camera.local_to_global << i.x(), j.x(), k.x(), this->_camera.pos.x(),
                i.y(), j.y(), k.y(), this->_camera.pos.y(),
                i.z(), j.z(), k.z(), this->_camera.pos.z(),
                0, 0, 0, 1;
    }


    std::vector<std::shared_ptr<Object>> _objs{};
    std::shared_ptr<BVH> _bvh{nullptr};

    // 场景中所有的发光体
    struct {
        std::vector<std::shared_ptr<Object>> objs{};
        float total_area{0.f};
    } _emit;

};


#endif //RENDER_DEBUG_SCENE_H
