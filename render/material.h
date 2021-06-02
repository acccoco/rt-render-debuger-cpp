
#ifndef RENDER_DEBUG_METERIAL_H
#define RENDER_DEBUG_METERIAL_H

#include <Eigen/Core>


enum class MatAttrType {
    None, Value, map
};


/* 材质的定义，主要负责计算 BRDF */
class Material {
public:

    /* phong shading BRDF */
    float brdf_phong(const Eigen::Vector3f &wi, const Eigen::Vector3f &wo) {
        float k_f;
        switch (diffuse_type) {
            case MatAttrType::Value:
                k_f = diffuse_value;
                break;
            default:
                k_f = 0.f;
        }
        return k_f / M_PI;
    }

    /* micro surface shading BRDF */
    float brdf_ms(const Eigen::Vector3f &wi, const Eigen::Vector3f &wo);

    MatAttrType diffuse_type = MatAttrType::Value;
    float diffuse_value;
};

#endif //RENDER_DEBUG_METERIAL_H
