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
        int col{}, row{};
        Ray ray;
    };

    /* 渲染一个像素得到的结果 */
    struct RenderPixelResult {
        int col, row;
        std::vector<std::deque<PathNode>> path_list; /* 每个像素对应的光路 */
    };

    using PixelType = std::array<unsigned char, 3>;

    /* 光线在物体上反射时，为了防止再与自身相交，让反射点沿法线偏离一定的距离 */
    static inline const float OFFSET = 0.01f;
    static inline const float RussianRoulette = 0.8f;   /* 俄罗斯轮盘赌的概率 */

    /* 渲染前的准备步骤：指定需要渲染的场景，以及 spp */
    static void init(const std::shared_ptr<Scene> &scene, int spp) {
        /* 创建 framebuffer，设置背景色为黑色 */
        framebuffer = std::vector<PixelType>(scene->screen_width() * scene->screen_width(),
                                             PixelType{0, 0, 0});
        _scene = scene;
        _spp = spp;
    }

    /**
     * 使用单线程来渲染场景
     * 会将详细的路径信息写入数据库，将像素信息写入 framebuffe 里面
     * @param db_path 存放光路信息的数据库
     */
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

    /* task：渲染一个像素 */
    static std::shared_ptr<RenderPixelResult> jobRenderOnePixel(const RenderPixelTask &task);

    /* 使用渲染得到的结果来绘制 framebuffer */
    static void drawFrameBuffer(const std::shared_ptr<RenderPixelResult> &res);

    /* 将一个像素对应的多个光线路径写入数据库 */
    static inline void insert_pixel_ray(sqlite3 *db, const RenderPixelResult &res) {
        for (auto &path : res.path_list) {
            PathSerialize::insertPath(db, res.row, res.col, path);
        }
    }

    /* 根据场景和渲染参数生成的渲染任务 */
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


public:
    static inline std::vector<PixelType> framebuffer; /* 渲染场景得到的帧缓冲 */

private:
    static inline int _spp = 16;                      /* 每个像素投射多少根光线 */
    static inline std::shared_ptr<Scene> _scene;      /* 需要渲染的场景 */
};


#endif //RENDER_DEBUG_RT_RENDER_H
