#ifndef RENDER_DEBUG_SCENE_H
#define RENDER_DEBUG_SCENE_H

#include <memory>

#include <Eigen/Eigen>

#include "bvh.h"
#include "object.h"
#include "intersection.h"


/**
 * 用于渲染的场景
 * 基本用法：
 *  Scene scene(...);
 *  scene.add_obj(...);     // 向场景中添加物体
 *  scene.build();          // 通过物体来建立场景的空间求交加速结构
 */
class Scene {
public:
    /**
     * 创建一个场景
     * @param screen_width 投影平面的宽度
     * @param screen_height 投影平面的高度
     * @param fov 摄像机的 FOV
     * @param camera_look_at 摄像机的朝向
     * @param camera_pos 摄像机在世界坐标系的坐标
     */
    Scene(int screen_width, int screen_height, float fov, const Eigen::Vector3f &camera_look_at,
          const Eigen::Vector3f &camera_pos)
            : _screen_width(screen_width), _screen_height(screen_height),
              _camera{Direction(camera_look_at), camera_pos, fov} {
        assert(screen_height > 0);
        assert(fov > 0.f && fov < 180.f);
        assert(camera_look_at.norm() > 0.f);
        assert(std::abs(camera_look_at.y()) < 0.9f);

        this->initInverseViewMatrix();
    }


    /* 从摄像机坐标系变换到 global 坐标系 */
    inline Eigen::Vector4f view_to_global(const Eigen::Vector4f &vec) {
        return this->_camera.view_matrix_inverse * vec;
    }

    /* 建立加速结构 */
    void build() {
        this->_bvh = BVH::build(this->_objs);
    }

    /* 向场景中添加一个物体 */
    void obj_add(const std::shared_ptr<Object> &obj);

    /* 光线是否和场景中的物体有交点；通过 BVH 的加速结构来判断 */
    [[nodiscard]] inline Intersection intersect(const Ray &ray) const {
        return _bvh->intersect(ray);
    }

    /**
     * 基于面积，对场景中的所有光源进行随机采样
     * @return [pdf, 采样点的信息]
     * @fixme 自己实现，会不会有什么问题
     */
    [[nodiscard]] std::tuple<float, Intersection> sample_light() const;

private:
    /**
     * 生成一个变换矩阵：将摄像机坐标系中的坐标变换到世界坐标系
     * 摄像机坐标系：摄像机朝向 -z
     */
    void initInverseViewMatrix();

private:
    int _screen_width, _screen_height;                  /* 投影平面的宽度与高度 */

    struct {
        Direction look_at;
        Eigen::Vector3f pos;
        float fov;
        Eigen::Matrix4f view_matrix_inverse;            /* 将摄像机坐标系变换到世界坐标系 */
    } _camera;

    std::vector<std::shared_ptr<Object>> _objs{};       /* 场景中所有的对象 */
    std::shared_ptr<BVH> _bvh{nullptr};                 /* 场景所有对象建立的加速结构 */

    // 场景中所有的发光体
    struct {
        std::vector<std::shared_ptr<Object>> objs{};
        float total_area{0.f};
    } _emit;

public:
    [[nodiscard]] inline int screen_width() const { return _screen_width; }

    [[nodiscard]] inline int screen_height() const { return _screen_height; }

    [[nodiscard]] inline float fov() const { return _camera.fov; }

    [[nodiscard]] inline Eigen::Vector3f camera_pos() const { return _camera.pos; };

    [[nodiscard]] inline const auto &emit() const { return _emit; }
};


#endif //RENDER_DEBUG_SCENE_H
