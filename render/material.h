
#ifndef RENDER_DEBUG_METERIAL_H
#define RENDER_DEBUG_METERIAL_H

#include <memory>

#include <Eigen/Eigen>

#include "ray.h"


// =========================================================
// 一些常用的颜色
// =========================================================
const Eigen::Vector3f Color_Gray{0.7f, 0.7f, 0.7f};
const Eigen::Vector3f color_cornel_white{0.725f, 0.71f, 0.68f};
const Eigen::Vector3f color_cornel_red{0.63f, 0.065f, 0.05f};
const Eigen::Vector3f color_cornel_green{0.14f, 0.45f, 0.091f};
const Eigen::Vector3f color_cornel_light{47.8348f, 38.5664f, 31.0808f};


/* 材质的定义，主要负责计算 BRDF */
class Material {
public:

    /* 材质的类型 */
    enum class MaterialType {
        Diffuse, Emission,
    };

    /* 工厂函数：创建一个灰色的漫反射材质 */
    static inline std::shared_ptr<Material> diffuse_mat() {
        return std::make_shared<Material>();
    }

    /* 默认材质，灰色的漫反射 */
    Material()
            : _mat_type(MaterialType::Diffuse),
              _diffuse(Color_Gray),
              _is_emission(false),
              _emission(0.f, 0.f, 0.f) {}

    /* 指定材质类型与颜色值 */
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

    /* BRDF：phong shading */
    [[nodiscard]] Eigen::Vector3f brdf_phong(const Direction &wi, const Direction &wo, const Direction &N) const {

        if (N.get().dot(wi.get()) < 0.f || N.get().dot(wo.get()) < 0.f)
            return Eigen::Vector3f{0.f, 0.f, 0.f};

        return _diffuse / M_PI;
    }

    /* BRDF：micro surface shading */
    Eigen::Vector3f brdf_ms(const Direction &wi, const Direction &wo, const Direction &N) {
        return {};
    }

    /**
     * 半球随机采样
     * @param N 采样表面的法线方向
     * @return [pdf，方向]
     */
    static std::tuple<float, Direction> sample_himsphere_random(const Direction &N);

    /**
     * 将 local 坐标转换为 global 坐标
     * @param N local 坐标系由 N 定义
     * @param local 需要转换的方向
     * @return
     */
    static Direction local_to_world(const Direction &N, const Direction &local);


private:
    MaterialType _mat_type;             /* 物体的材质类型 */
    Eigen::Vector3f _diffuse;           /* 材质的漫反射颜色值 */
    bool _is_emission;                  /* 当前材质是否是自发光的 */
    Eigen::Vector3f _emission;          /* 材质的发光值 */

public:
    [[nodiscard]] inline bool is_emission() const { return _is_emission; }

    [[nodiscard]] inline Eigen::Vector3f emission() const { return _emission; }

};

#endif //RENDER_DEBUG_METERIAL_H
