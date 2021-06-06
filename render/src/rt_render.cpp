
#include <iostream>

#include "../utils.h"
#include "../rt_render.h"


/**
 * 计算反射方程，对光源采样
 * @param inter
 * @param inter_light 和光源的交点
 * @param wi 物体到光源的射线
 * @param wo 光线射出物体的方向
 * @param pdf_light 当前采样的概率密度
 * @return
 */
inline Eigen::Vector3f reflect_equation_light(const Intersection &inter, const Intersection &inter_light,
                                              const Direction &wi, const Direction &wo, float pdf_light) {
    float dis_to_light = (inter_light.pos() - inter.pos()).norm();
    float dis_to_light2 = dis_to_light * dis_to_light;
    float cos_theta = std::max(0.f, inter.normal().get().dot(wi.get()));
    float cos_theta_1 = std::max(0.f, inter_light.normal().get().dot(-wi.get()));
    auto Li = inter_light.obj()->material().emission();
    auto BRDF = inter.obj()->material().brdf_phong(wi, wo, inter.normal());
    return Li.array() * BRDF.array() * cos_theta * cos_theta_1 / dis_to_light2 / pdf_light;
}


void RTRender::cast_ray_obj(const Ray &ray, const Intersection &inter, std::deque<PathNode> &path) {
    // 来到这里的一定是和不发光物体相交的
    assert(inter.happened());
    assert(!inter.obj()->material().is_emission());

    // 路径信息
    PathNode node;
    node.Lo = Eigen::Vector3f(0.f, 0.f, 0.f);
    node.wo = -ray.direction();
    node.inter = inter;

    // =========================================================
    // 1. 向光源投射光线
    // =========================================================
    RUN_ONCE {
        // 在场景中的光源进行随机采样
        auto[pdf_light, inter_light] = this->_scene->sample_light();

        // 如果场景中并没有光源：
        if (!inter_light.happened()) {
            node.set_light_inter(Eigen::Vector3f(0.f, 0.f, 0.f), Direction::zero(), Intersection::no_intersect());
            break;
        }
        assert(inter_light.obj()->material().is_emission());

        // 判断光源和物体之间是否有遮挡
        Ray ray_to_light{inter.pos(), inter_light.pos() - inter.pos()};
        Intersection inter_light_dir = _scene->intersect(ray_to_light);
        if ((inter_light_dir.pos() - inter_light.pos()).norm() > epsilon_4) {
            node.set_light_inter(Eigen::Vector3f(0.f, 0.f, 0.f), ray_to_light.direction(), inter_light_dir);
            break;
        }

        // 计算反射方程
        Eigen::Vector3f Lo_light = reflect_equation_light(inter, inter_light, ray_to_light.direction(),
                                                          -ray.direction(), pdf_light);

        // 添加路径信息
        node.Lo += Lo_light;
        node.set_light_inter(inter_light.obj()->material().emission(), ray_to_light.direction(), inter_light);
    };

    // =========================================================
    // 2. 向其他物体投射光线
    // =========================================================
    RUN_ONCE {
        // 俄罗斯轮盘赌测试
        float RR = random_float_get();
        if (RR > RussianRoulette) {
            node.set_obj_inter(RR, Direction::zero(), Intersection::no_intersect());
            break;
        }

        // 使用半球随机采样
        auto[pdf_obj, wi_obj] = Material::sample_himsphere_random(inter.normal());
        Ray ray_to_obj{inter.pos(), wi_obj};
        Intersection inter_with_obj = this->_scene->intersect(ray_to_obj);

        // 是否发生相交
        if (!inter_with_obj.happened()) {
            node.set_obj_inter(RR, wi_obj, inter_with_obj);
            break;
        }

        // 是否为发光体，已经对发光体进行过采样了
        if (inter_with_obj.obj()->material().is_emission()) {
            node.set_obj_inter(RR, wi_obj, inter_with_obj);
            break;
        }

        // 计算下一段光路
        RTRender::cast_ray_obj(ray_to_obj, inter_with_obj, path);
        assert(!path.empty());

        // 计算和物体相交的反射方程
        Eigen::Vector3f Li_obj = path.front().Lo;
        Eigen::Vector3f fr = inter_with_obj.obj()->material().brdf_phong(wi_obj, -ray.direction(), inter.normal());
        float cos_theta = std::max(0.f, inter.normal().get().dot(wi_obj.get()));
        Eigen::Vector3f Lo_object = Li_obj.array() * fr.array() * cos_theta / pdf_obj / RussianRoulette;

        // 添加路径信息
        node.Lo += Lo_object;
        node.set_obj_inter(RR, wi_obj, inter_with_obj);
    }

    path.push_front(std::move(node));
}
