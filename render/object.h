#ifndef RENDER_DEBUG_OBJECT_H
#define RENDER_DEBUG_OBJECT_H

#include "bounding_box.h"


class Object {
public:

private:
    virtual Intersection intersect() = 0;
    virtual BoundingBox bdbox_get() = 0;
};

#endif //RENDER_DEBUG_OBJECT_H
