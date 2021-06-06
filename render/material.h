
#ifndef RENDER_DEBUG_METERIAL_H
#define RENDER_DEBUG_METERIAL_H

#include <Eigen/Eigen>

#include "ray.h"


enum class MaterialType {
    Diffuse,
    Emission,
};


// =========================================================
// 一些常用的颜色
// =========================================================
const Eigen::Vector3f Color_Gray{0.7f, 0.7f, 0.7f};
const Eigen::Vector3f color_cornel_white{0.725f, 0.71f, 0.68f};
const Eigen::Vector3f color_cornel_red{0.63f, 0.065f, 0.05f};
const Eigen::Vector3f color_cornel_green{0.14f, 0.45f, 0.091f};
const Eigen::Vector3f color_cornel_light{8.0f * Eigen::Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) +
                                         15.6f * Eigen::Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) +
                                         18.4f * Eigen::Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f)};


/* 材质的定义，主要负责计算 BRDF */
class Material {
public:

    // 默认材质，灰色的漫反射
    Material()
            : _mat_type(MaterialType::Diffuse),
              _diffuse(Color_Gray),
              _is_emission(false),
              _emission(0.f, 0.f, 0.f) {}

    Material(MaterialType mat_type, const Eigen::Vector3f &value)
            : _mat_type(mat_type) {
        assert(mat_type == MaterialType::Diffuse || mat_type == MaterialType::Emission);
        switch (mat_type) {
            case MaterialType::Diffuse:
                _diffuse = value;
                _is_emission = false;
                break;
            case MaterialType::Emission:
                _emission = value;
                _is_emission = true;
                break;
            default:
                throw std::runtime_error("never");
        }
    }

    // 灰色的漫反射材质
    static inline std::shared_ptr<Material> diffuse_mat() {
        return std::make_shared<Material>();
    }


    // 将材质设为发光的
    void set_emission(const Eigen::Vector3f &value) {
        this->_mat_type = MaterialType::Emission;
        this->_is_emission = true;
        this->_emission = value;
    }


    // 将材质设为漫反射的
    void set_diffuse(const Eigen::Vector3f &value) {
        this->_mat_type = MaterialType::Diffuse;
        this->_is_emission = false;
        this->_diffuse = value;
    }

    // =========================================================
    // BRDF
    // =========================================================
    /* phong shading BRDF */
    Eigen::Vector3f brdf_phong(const Direction &wi, const Direction &wo, const Direction &N) const {

        if (N.get().dot(wi.get()) < 0.f || N.get().dot(wo.get()) < 0.f)
            return Eigen::Vector3f{0.f, 0.f, 0.f};

        return _diffuse / M_PI;
    }

    /* micro surface shading BRDF */
    Eigen::Vector3f brdf_ms(const Direction &wi, const Direction &wo, const Direction &N) {
        return {};
    }


    // =========================================================
    // 属性
    // =========================================================
    inline bool is_emission() const {
        return this->_is_emission;
    }

    inline Eigen::Vector3f emission() const {
        return this->_emission;
    }


    // =========================================================
    // 半球随机采样
    // =========================================================
    static std::tuple<float, Direction> sample_himsphere_random(const Direction &N);

    static Direction local_to_world(const Direction &N, const Direction &local);


private:
    // todo 这里同时有 mat type 和 is emission，考虑去掉一个
    MaterialType _mat_type;
    Eigen::Vector3f _diffuse;
    bool _is_emission;
    Eigen::Vector3f _emission;

};

#endif //RENDER_DEBUG_METERIAL_H
