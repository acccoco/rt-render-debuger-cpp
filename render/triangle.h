#ifndef RENDER_DEBUG_TRIANGLE_H
#define RENDER_DEBUG_TRIANGLE_H

#include <Eigen/Eigen>
#include "bvh.h"
#include "object.h"


class Triangle : public Object {
public:
    Triangle(Eigen::Vector3f v0, Eigen::Vector3f v1,
             Eigen::Vector3f v2);

private:
    Eigen::Vector3f A, B, C;
    Eigen::Vector3f face_normal;
};


class MeshTriangle : public Object {

};

#endif //RENDER_DEBUG_TRIANGLE_H
