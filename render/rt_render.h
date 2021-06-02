#ifndef RENDER_DEBUG_RT_RENDER_H
#define RENDER_DEBUG_RT_RENDER_H

#include <memory>
#include <deque>
#include <Eigen/Eigen>
#include "object.h"
#include "ray.h"


// 多线程 workder 的任务
typedef Eigen::Vector2i Task;

// 多线程 workder 的计算结果
class Result {
public:
    Task task;
    Eigen::Vector3f radiance;
};



class RTRender {
public:

    // 投射出一根光线，递归地计算 Radiance
    Eigen::Vector3f cast_ray(const Ray &ray);

    void rend();

    // 更新进度
    void process_update();

    // 生成新的任务
    void task_produce();

    // 处理结果
    void result_process();

private:
    const float RussianRoulette = 0.8f;
    const int spp = 16;

    // 存放多线程任务的双端队列
    std::deque<Task> task_queue;

    // 存放对线程结果的双端队列
    std::deque<Result> result_queue;

    std::vector<float> frame_buffer;
};

#endif //RENDER_DEBUG_RT_RENDER_H
