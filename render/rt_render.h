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
#include "utils.h"
#include "ray_path.h"
#include "ray_path_serialize.h"


/**
 * 渲染场景，基本流程为：
 *  RTRender::init(...);
 *  RTRender::render();
 *  RTRender::write_to_file();
 */
class RTRender {
public:
    /* 渲染一个像素的任务 */
    struct RenderPixelTask {
        int col, row;
        Ray ray;
    };

    /* 渲染一个像素得到的结果 */
    struct RenderPixelResult {
        int col, row;
        std::vector<std::deque<PathNode>> path_list; /* 每个像素对应的光路 */
    };

    using PixelType = std::array<unsigned char, 3>;

    static inline std::vector<PixelType> framebuffer; /* 渲染场景得到的帧缓冲 */

    /* 使用 scene 初始化渲染器，准备渲染 */
    static void init(const std::shared_ptr<Scene> &scene, int spp) {
        /* 创建 framebuffer，设置背景色为黑色 */
        framebuffer = std::vector<PixelType>(scene->screen_width() * scene->screen_width(),
                                             PixelType{0, 0, 0});
        _scene = scene;
        _spp = spp;
    }

    /* 渲染一个像素 */
    static inline std::shared_ptr<RenderPixelResult> job_render_one_pixel(const RenderPixelTask &task) {
        std::vector<std::deque<PathNode>> path_list;
        for (int i = 0; i < _spp; ++i) {
            path_list.push_back(cast_ray(task.ray));
        }
        return std::shared_ptr<RenderPixelResult>(new RenderPixelResult{
                task.col, task.row,
                std::move(path_list)});
    }

    /* 对每个像素的渲染结果进行后处理 */
    static inline void process_render_result(const std::shared_ptr<RenderPixelResult> &res) {
        assert(res->path_list.size() == _spp);

        /* 得到最终的 radiance */
        Eigen::Vector3f radiance{0.f, 0.f, 0.f};
        for (auto &path : res->path_list) {
            radiance += path[0].Lo / _spp;
        }

        /* 将结果写入 framebuffer */
        framebuffer[res->row * _scene->screen_width() + res->col] = gamma_correct(radiance);
    }

    /* 将一个像素对应的多个光线路径写入数据库 */
    static inline void insert_pixel_ray(sqlite3 *db, const RenderPixelResult &res) {
        for (auto &path : res.path_list) {
            PathSerialize::write_to_sqlite(db, res.row, res.col, path);
        }
    }

    /* 使用单线程来渲染场景 */
    static void render_single_thread(const std::string &db_path);

    /**
     * 使用多线程来渲染场景
     * @param worker_buffer_size worker 缓存的大小，缓存越小，加锁越频繁
     * @param worker_sleep_ms 如果任务列表空了，worker 就会 sleep，该参数可以设置 sleep 的时间
     * @param master_process_interval 主线程处理结果的时间间隔，时间越小，加锁越频繁
     */
    static void render_multi_thread(const std::string &db_path, int worker_cnt, int worker_buffer_size,
                                    int worker_sleep_ms, int master_process_interval);

    /* 将 framebuffer 写入 ppm 文件中 */
    static void write_to_file(const std::vector<PixelType> &buffer, const char *file_path,
                              int width, int height);

private:
    /* 根据场景和渲染参数生成渲染任务 */
    static std::vector<RenderPixelTask> _prepare_render_task(const std::shared_ptr<Scene> &scene);

    /* 向场景投射一根光线，得到路径信息 */
    static std::deque<PathNode> cast_ray(const Ray &ray);

    /**
     * 向物体投射出一根光线，递归地得到路径信息
     * @param inter 与该物体的交点
     * @param [out]path 将结果写入该参数
     * @note 需要保证该光线和物体相交，且物体不是发光的
     */
    static void cast_ray_recursive(const Ray &ray, const Intersection &inter, std::deque<PathNode> &path);

    /* 将 [0, 1] 范围的 Radiance 值进行 Gamma 矫正，并转换为 [0, 255] 的颜色值 */
    static inline PixelType gamma_correct(const Eigen::Vector3f &radiance) {
        unsigned char x = (unsigned char) (255 * std::pow(std::clamp(radiance.x(), 0.f, 1.f), 0.6f));
        unsigned char y = (unsigned char) (255 * std::pow(std::clamp(radiance.y(), 0.f, 1.f), 0.6f));
        unsigned char z = (unsigned char) (255 * std::pow(std::clamp(radiance.z(), 0.f, 1.f), 0.6f));
        return {x, y, z};
    }

    static inline const float RussianRoulette = 0.8f; /* 俄罗斯轮盘赌的概率 */
    static inline int _spp = 16;                      /* 每个像素投射多少根光线 */
    static inline std::shared_ptr<Scene> _scene;      /* 需要渲染的场景 */
};

#endif //RENDER_DEBUG_RT_RENDER_H
