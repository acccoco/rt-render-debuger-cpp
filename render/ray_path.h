//
// Created by 漆正杰 on 2021/6/21.
//

#ifndef RENDER_DEBUG_RAY_PATH_H
#define RENDER_DEBUG_RAY_PATH_H

#include <Eigen/Eigen>

#include "ray.h"
#include "intersection.h"


// 路径追踪的一个节点，路径从摄像机出发
struct PathNode {

    // 设置和光源的相交信息
    inline void
    set_light_inter(const Eigen::Vector3f &Li_light, const Direction &_wi_light, const Intersection &_inter_light) {
        this->from_light.Li_light = Li_light;
        this->from_light.wi_light = _wi_light;
        this->from_light.inter_light = _inter_light;
    }

    // 设置和物体的相交信息
    inline void
    set_obj_inter(float RR, const Direction &_wi_object, const Intersection &_inter_obj,
                  const Eigen::Vector3f &Li_obj = {0.f, 0.f, 0.f}) {
        this->from_obj.RR = RR;
        this->from_obj.Li_obj = Li_obj;
        this->from_obj.wi_obj = _wi_object;
        this->from_obj.inter_obj = _inter_obj;
    }


    // =========================================================================
    // member field
    // =========================================================================

    // 出射光线的信息
    Eigen::Vector3f Lo{0.f, 0.f, 0.f};          /* 出射光线的辐照度 */
    Direction wo = Direction::zero();                       /* 出射光线的方向 */
    Eigen::Vector3f pos_out{0.f, 0.f, 0.f};     /* 出射光线的端点 */
    Intersection inter = Intersection::no_intersect();      /* 是否和物体有交点 */

    // 来自光源的入射光线
    struct {
        Eigen::Vector3f Li_light{0.f, 0.f, 0.f};
        Direction wi_light = Direction::zero();
        Intersection inter_light = Intersection::no_intersect();
    } from_light;

    // 来自物体的入射光线
    struct {
        float RR{-1.f};
        Eigen::Vector3f Li_obj{0.f, 0.f, 0.f};
        Direction wi_obj = Direction::zero();
        Intersection inter_obj = Intersection::no_intersect();
    } from_obj;
};

#endif //RENDER_DEBUG_RAY_PATH_H
