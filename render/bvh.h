#ifndef RENDER_DEBUG_BVH_H
#define RENDER_DEBUG_BVH_H

#include <memory>
#include "object.h"

// BVH 的树节点
class BVH {
public:
    std::shared_ptr<BVH> build();

private:
    std::shared_ptr<BVH> left_child;
    std::shared_ptr<BVH> right_child;

    std::shared_ptr<Object> object;
};

#endif //RENDER_DEBUG_BVH_H
