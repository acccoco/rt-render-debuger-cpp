#ifndef RENDER_DEBUG_SCENE_H
#define RENDER_DEBUG_SCENE_H

#include <memory>
#include <Eigen/Eigen>
#include "object.h"
#include "bvh.h"

class Scene {
public:

    void obj_add(const std::shared_ptr<Object> &obj);

private:
    int view_height;
    float aspect;
    float fov;

    std::shared_ptr<BVH> bvh;

    std::vector<std::shared_ptr<Object>> objs;

};

#endif //RENDER_DEBUG_SCENE_H
