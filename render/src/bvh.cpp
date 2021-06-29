#include "bvh.h"
#include "utils.h"


std::tuple<std::vector<std::shared_ptr<Object>>, std::shared_ptr<Object>, std::vector<std::shared_ptr<Object>>>
find_kth_obj(const std::vector<std::shared_ptr<Object>> &objs, unsigned int k, ExtensionDir dir) {
    auto n_size = objs.size();
    assert(k < n_size);

    // 只有一个图形的情况
    if (n_size == 1) {
        return {std::vector<std::shared_ptr<Object>>{}, objs[0], std::vector<std::shared_ptr<Object>>{}};
    }

    // 以第一个 Obejct 为参考，来切分 objects
    auto flag_obj = objs[0];
    std::vector<std::shared_ptr<Object>> less, greater;
    for (size_t i = 1; i < n_size; ++i) {
        auto obj = objs[i];
        bool is_less;
        switch (dir) {
            case ExtensionDir::X:
                is_less = obj->bounding_box().center().x() <= flag_obj->bounding_box().center().x();
                break;
            case ExtensionDir::Y:
                is_less = obj->bounding_box().center().y() <= flag_obj->bounding_box().center().y();
                break;
            case ExtensionDir::Z:
                is_less = obj->bounding_box().center().z() <= flag_obj->bounding_box().center().z();
                break;
            default:
                throw std::runtime_error("never");
        }
        if (is_less) less.push_back(obj);
        else greater.push_back(obj);
    }

    // 递归地查找下去
    if (less.size() == k) {
        // [less], flag, [greater]
        return {less, flag_obj, greater};
    } else if (less.size() > k) {
        // [[less_less], less_flag, [less_greater]], flag, [greater]
        auto[less_less, less_flag, less_greater] = find_kth_obj(less, k, dir);
        less_greater.push_back(flag_obj);
        less_greater.insert(less_greater.end(), greater.begin(), greater.end());
        return {less_less, less_flag, less_greater};
    } else {
        // [less], flag, [[greater_less], greater_flag, [greater_greater]]
        auto[greater_less, greater_flag, greater_greater] = find_kth_obj(greater, k - less.size() - 1, dir);
        less.push_back(flag_obj);
        less.insert(less.end(), greater_less.begin(), greater_less.end());
        return {less, greater_flag, greater_greater};
    }
}


std::shared_ptr<BVH> BVH::build(const std::vector<std::shared_ptr<Object>> &objs) {
    auto obj_size = objs.size();
    assert(obj_size > 0);

    // 只有一个 object 的情况
    if (obj_size == 1) {
        return std::make_shared<BVH>(objs[0]->bounding_box(), objs[0]->area(), objs[0]);
    }

    // 计算包围盒
    BoundingBox box;
    for (const auto &obj : objs) {
        box.unionOp(obj->bounding_box());
    }

    // 找到位于中间靠前的 object，可以确保 [less + k_th] 和 greater 的元素数量都 >= 1
    auto max_ext_dir = box.maxExtension();
    auto[less, k_th, greater] = find_kth_obj(objs, (obj_size - 1) / 2, max_ext_dir);
    less.push_back(k_th);
    auto lchild = build(less);
    auto rchild = build(greater);

    return std::make_shared<BVH>(box, lchild->_area + rchild->_area,
                                 lchild, rchild);
}


Intersection BVH::intersect(const Ray &ray) const {

    // 没有发生相交
    if (!this->bounding_box().isIntersect(ray)) {
        return Intersection::no_intersect();
    }

    // 当前节点是叶子节点
    if (this->_object) {
        assert(!this->_lchild && !this->_rchild);
        // return Intersection::intersect(this->_object, ray);
        return _object->intersect(ray);
    }

    // 判断子节点是否发生相交
    assert(this->_lchild && this->_rchild);
    auto l_inter = _lchild->intersect(ray);
    auto r_inter = _rchild->intersect(ray);

    /**
     * 分为 4 种情况讨论：
     *  1. 只有左子有交点：返回左子的交点
     *  2. 只有右子有交点，同上
     *  3. 左右子都有交点，返回距 origin 近的交点
     *  4. 左右子都没有交点，返回不相交
     */
    int res = (l_inter.happened() ? 1 : 0) + (r_inter.happened() ? 2 : 0);
    switch (res) {
        case 1:     // 只有左子相交
            return l_inter;
        case 2:     // 只有右子相交
            return r_inter;
        case 3:     // 左右子都相交，最近的
            return l_inter.t_near() < r_inter.t_near() ? l_inter : r_inter;
        default:    // 没有发生相交
            return Intersection::no_intersect();
    }
}

Intersection BVH::sample_obj(float area_threshold) {
    assert(this->_area - area_threshold > -epsilon_4);

    // 当前节点是叶子节点
    if (this->_object) {
        return _object->obj_sample(area_threshold);
        // return Object::sample_obj(this->_object, area_threshold);
    }

    // 去左右子树找
    if (this->_lchild->_area > area_threshold) {
        return this->_lchild->sample_obj(area_threshold);
    } else {
        return this->_rchild->sample_obj(area_threshold - this->_lchild->_area);
    }
}
