
#ifndef RENDER_DEBUG_UTILS_H
#define RENDER_DEBUG_UTILS_H

#include <cmath>
#include <random>

#include <Eigen/Eigen>


// =========================================================
// 基于标准库的误差定义
// 注：数字表示精度级别，小数点后多少位
// =========================================================
const inline float epsilon_7 = std::numeric_limits<float>::epsilon();
const inline float epsilon_6 = epsilon_7 * 10.f;
const inline float epsilon_5 = epsilon_7 * 100.f;
const inline float epsilon_4 = epsilon_7 * 1000.f;
const inline float epsilon_3 = epsilon_7 * 10000.f;
const inline float epsilon_2 = epsilon_7 * 100000.f;
const inline float epsilon_1 = epsilon_7 * 1000000.f;
const inline float epsilon_38 = std::numeric_limits<float>::min();


// 浮点数相等的判断，误差是否在小数点后 4 位
#define EQUAL_F4(a, b) (std::abs((a) - (b)) < epsilon_4)



// [0, 1] 均匀分布的 float
inline float random_float_get()
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    return dist(rng);
}


// 随机点，范围是 [0, 1]^3
inline Eigen::Vector3f random_point_get() {
    return Eigen::Vector3f(random_float_get(), random_float_get(), random_float_get());
}


// 随机点，范围是 [min, max]^3
inline Eigen::Vector3f random_point_get(float min, float max) {
    assert(min < max);
    auto delta = max - min;
    return Eigen::Vector3f(random_float_get() * delta + min,
                           random_float_get() * delta + min,
                           random_float_get() * delta + min);
}


// 一个花括号作用域，可以通过 break 跳出
#define RUN_ONCE for(int __u_n_i_q_u_e__v_a_r__ = 1; __u_n_i_q_u_e__v_a_r__ > 0; __u_n_i_q_u_e__v_a_r__--)


// 循环的简便写法
#define LOOP(times) for(int __u_n_i_q_u_e__v_a_r__ = 0; __u_n_i_q_u_e__v_a_r__ < (times); ++__u_n_i_q_u_e__v_a_r__)

#endif //RENDER_DEBUG_UTILS_H
