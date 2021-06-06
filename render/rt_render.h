#ifndef RENDER_DEBUG_RT_RENDER_H
#define RENDER_DEBUG_RT_RENDER_H


#include <deque>
#include <mutex>
#include <array>
#include <memory>
#include <thread>
#include <vector>
#include <chrono>

#include <Eigen/Eigen>
#include <fmt/format.h>

#include "ray.h"
#include "scene.h"
#include "object.h"
#include "config.h"
#include "task.h"


// todo 整理一下这里的代码，太乱了

// 路径追踪的一个节点，路径从摄像机出发
struct PathNode {

    // 设置和光源的相交信息
    inline void
    set_light_inter(const Eigen::Vector3f &_Li_light, const Direction &_wi_light, const Intersection &_inter_light) {
        this->from_light.Li_light = _Li_light;
        this->from_light.wi_light = _wi_light;
        this->from_light.inter_light = _inter_light;
    }

    // 设置和物体的相交信息
    inline void
    set_obj_inter(float _RR, const Direction &_wi_object, const Intersection &_inter_obj) {
        this->from_obj.RR = _RR;
        this->from_obj.wi_obj = _wi_object;
        this->from_obj.inter_obj = _inter_obj;
    }

    // 出射光线的信息
    Eigen::Vector3f Lo{0.f, 0.f, 0.f};
    Direction wo = Direction::zero();
    Intersection inter = Intersection::no_intersect();

    // 来自光源的入射光线
    struct {
        Eigen::Vector3f Li_light{0.f, 0.f, 0.f};
        Direction wi_light = Direction::zero();
        Intersection inter_light = Intersection::no_intersect();
    } from_light;

    // 来自物体的入射光线
    struct {
        float RR{-1.f};
        Direction wi_obj = Direction::zero();
        Intersection inter_obj = Intersection::no_intersect();
    } from_obj;
};


// 每个像素的渲染任务
struct PixelTask {
    int screen_x, screen_y;
    Ray ray;
};

// 数据后处理的结果
struct PixelResult {
    int screen_x, screen_y;
    std::vector<std::deque<PathNode>> path_list;    // 每个像素对应的光路
};


class RTRender {
public:
    RTRender(const std::shared_ptr<Scene> &scene) : _scene(scene) {
        _frame_buffer = std::vector<std::vector<std::array<unsigned char, 3>>>(
                scene->screen_height(), std::vector<std::array<unsigned char, 3>>(
                        scene->screen_width(), std::array<unsigned char, 3>{0, 0, 0}));
    }

    // 准备渲染需要的任务
    std::deque<PixelTask> prepare_tasks() {
        std::deque<PixelTask> task_queue;

        // 设视平面位于摄像机前方，距离为 1
        float view_height = 2.f * std::tan(_scene->fov() / 2.f / 180.f * M_PI);
        float view_width = view_height / _scene->screen_height() * _scene->screen_height();

        // 对于每个像素点的中心点，建立任务
        for (int i = 0; i < _scene->screen_height(); ++i) {
            for (int j = 0; j < _scene->screen_width(); ++j) {
                float view_x = ((j + 0.5f) / _scene->screen_width() - 0.5f) * view_width;
                float view_y = (0.5f - (i + 0.5f) / _scene->screen_height()) * view_height;

                Eigen::Vector4f dir_global = _scene->local_to_global({view_x, view_y, -1.f, 0.f});
                Ray ray(_scene->camera_pos(), dir_global.head(3));

                task_queue.push_back(PixelTask{j, i, ray});
            }
        }

        return task_queue;
    }

    // 线程计算任务
    std::shared_ptr<PixelResult> job_bee(const PixelTask &task) {
        std::vector<std::deque<PathNode>> path_list;
        for (int i = 0; i < _spp; ++i) {
            path_list.push_back(cast_ray(task.ray));
        }
        return std::shared_ptr<PixelResult>(new PixelResult{task.screen_x, task.screen_y, std::move(path_list)});
    }

    // 数据处理函数
    // todo 把这个改成引用类型的
    void job_ant(const std::shared_ptr<PixelResult> &result) {
        assert(result->screen_y < _frame_buffer.size());
        assert(result->screen_x < _frame_buffer[0].size());
        assert(result->path_list.size() == _spp);

        // 将结果写入 framebuffer 中
        Eigen::Vector3f radiance{0.f, 0.f, 0.f};
        for (auto &path : result->path_list) {
            assert(path.size() > 0);        // 一定有路径信息的
            radiance += path[0].Lo / _spp;
        }
        _frame_buffer[result->screen_y][result->screen_x] = gamma_correct(radiance);
    }

    // 渲染整个场景
    void render_single_thread(int spp) {
        assert(spp > 0);
        this->_spp = spp;

        std::deque<PixelTask> tasks = prepare_tasks();
        std::vector<std::shared_ptr<PixelResult>> results;

        // bee 过程
        for (auto &task: tasks) {
            results.push_back(job_bee(task));
        }

        // ant 过程
        for (auto &result : results) {
            job_ant(result);
        }

        // 写入文件
        write_to_file(_frame_buffer, RT_RES);
    }


    /* 多线程渲染 */
    void render(int spp) {
        assert(spp > 0);
        this->_spp = spp;

        /* 任务队列 */
        std::deque<PixelTask> tasks = prepare_tasks();

        auto bee_job = [this](const PixelTask &task) { return this->job_bee(task); };
        auto ant_job = [this](const std::shared_ptr<PixelResult> &result) { this->job_ant(result); };
        Giraffe<PixelTask, std::shared_ptr<PixelResult>> giraffe(4, 400, 400, 200,
                                                                 bee_job, ant_job, tasks);

        giraffe.run();

        // 写入文件
        write_to_file(_frame_buffer, RT_RES);
    }


    // 投射一根光线，计算路径信息
    std::deque<PathNode> cast_ray(const Ray &ray) {
        Intersection inter = _scene->intersect(ray);

        // 没有发生相交
        if (!inter.happened()) {
            PathNode node;
            node.Lo = Eigen::Vector3f(0.f, 0.f, 0.f);
            node.wo = -ray.direction();
            node.inter = inter;
            return {std::move(node)};
        }

        // 与发光体相交
        if (inter.obj()->material().is_emission()) {
            PathNode node;
            node.Lo = inter.obj()->material().emission();
            node.wo = -ray.direction();
            node.inter = inter;
            return {std::move(node)};
        }

        // 与物体相交
        std::deque<PathNode> path;
        cast_ray_obj(ray, inter, path);
        return path;
    }

    // 投射出一根光线，递归地计算 Radiance
    void cast_ray_obj(const Ray &ray, const Intersection &inter, std::deque<PathNode> &path);

    // 将以 1 为参考的 Radiance 值进行 Gamma 矫正，并转换为 [0, 255] 的颜色值
    static inline std::array<unsigned char, 3> gamma_correct(const Eigen::Vector3f &radiance) {
        unsigned char x = (unsigned char) (255 * std::pow(clamp(0.f, 1.f, radiance.x()), 0.6f));
        unsigned char y = (unsigned char) (255 * std::pow(clamp(0.f, 1.f, radiance.y()), 0.6f));
        unsigned char z = (unsigned char) (255 * std::pow(clamp(0.f, 1.f, radiance.z()), 0.6f));

        return {x, y, z};
    }


    /**
     * 将 framebuffer 写入 ppm 文件中
     * @param buffer
     * @param file_path
     */
    static void
    write_to_file(const std::vector<std::vector<std::array<unsigned char, 3>>> &buffer, const char *file_path) {
        unsigned height = buffer.size();
        unsigned width = buffer[0].size();
        assert(height > 0 && width > 0);

        FILE *fp = fopen(file_path, "wb");
        fprintf(fp, "P6\n%d %d\n255\n", width, height);

        for (unsigned i = 0; i < height; ++i) {
            for (unsigned j = 0; j < width; ++j) {
                fwrite(&buffer[i][j][0], sizeof(unsigned char), 3, fp);
            }
        }
        fclose(fp);
    }

private:
    /**
     * 俄罗斯轮盘赌的概率
     * 次数期望的计算公式为：RR / (1 - RR) ^2
     * 当 RR = 0.8 时，期望弹射 20 次
     */
    const float RussianRoulette = 0.8f;
    int _spp = 16;

    std::shared_ptr<Scene> _scene;

    // 屏幕左上角是原点
    std::vector<std::vector<std::array<unsigned char, 3>>> _frame_buffer;
};

#endif //RENDER_DEBUG_RT_RENDER_H
