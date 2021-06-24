#include "rt_render.h"

#include <iostream>



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


// todo 这个值可以通过后续的实验确定
//  将 offset 以及误差相关的计算写到文档里面
static const float OFFSET = 0.01f;


void RTRender::cast_ray_recursive(const Ray &ray, const Intersection &inter, std::deque<PathNode> &path) {
    assert(inter.happened());
    assert(!inter.obj()->material().is_emission());

    /**
     * 入射光线主要有两个来源：
     *  1. 来自于光源（通过对光源的采样来计算这一部分的值）
     *  2. 来自于其他物体（通过在半球空间采样来计算这一部分的值）
     */

    /* 路径信息 */
    PathNode node;
    node.Lo = Eigen::Vector3f(0.f, 0.f, 0.f);
    node.wo = -ray.direction();
    node.pos_out = ray.origin();
    node.inter = inter;

    // =========================================================
    // 1. 向光源投射光线
    // =========================================================
    RUN_ONCE {
        // 在场景中的光源进行随机采样
        auto[pdf_light, inter_light] = _scene->sample_light();

        // 如果场景中并没有光源：
        if (!inter_light.happened()) {
            node.set_light_inter(Eigen::Vector3f(0.f, 0.f, 0.f), Direction::zero(), Intersection::no_intersect());
            break;
        }
        assert(inter_light.obj()->material().is_emission());

        // 判断到光源采样点的路上是否有被遮挡
        // 构造光线时，让原点在法线方向上又一个偏移，防止与自身相交
        Ray ray_to_light{inter.pos() + inter.normal().get() * OFFSET, inter_light.pos() - inter.pos()};
        // 由于光线的原点有了偏移，终点也会出现偏移
        float delta = 0;
        {
            float _cos_theta = inter.normal().get().dot(ray_to_light.direction().get());
            float _cos_theta1 = inter_light.normal().get().dot(-ray_to_light.direction().get());
            delta = OFFSET * std::sqrt(1 - _cos_theta * _cos_theta) / _cos_theta1;
        }
        Intersection inter_light_dir = _scene->intersect(ray_to_light);
        if ((inter_light_dir.pos() - inter_light.pos()).norm() > delta + epsilon_4) {
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
        // 让光线原点沿法线偏移
        Ray ray_to_obj{inter.pos() + inter.normal().get() * OFFSET, wi_obj};
        Intersection inter_with_obj = _scene->intersect(ray_to_obj);

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
        RTRender::cast_ray_recursive(ray_to_obj, inter_with_obj, path);
        assert(!path.empty());

        // 计算和物体相交的反射方程
        Eigen::Vector3f Li_obj = path.front().Lo;
        // fixme: 这里将 inter_with_obj 改成了 inter
        Eigen::Vector3f fr = inter.obj()->material().brdf_phong(wi_obj, -ray.direction(), inter.normal());
        float cos_theta = std::max(0.f, inter.normal().get().dot(wi_obj.get()));
        Eigen::Vector3f Lo_object = Li_obj.array() * fr.array() * cos_theta / pdf_obj / RussianRoulette;

        // 添加路径信息
        node.Lo += Lo_object;
        node.set_obj_inter(RR, wi_obj, inter_with_obj, Li_obj);
    }

    path.push_front(std::move(node));
}

std::deque<PathNode> RTRender::cast_ray(const Ray &ray) {
    Intersection inter = _scene->intersect(ray);

    /**
     * 从摄像机射出一根光线，有三种情况
     *  1. 不和任何物体相交
     *  2. 和发光体相交，返回相交的信息
     *      （因为这里使用了单独的对光源采样，所以需要单独处理这种情况）
     *  3. 和不发光的物体相交，递归地计算光路
     */

    /* 不和任何物体相交 */
    if (!inter.happened()) {
        PathNode node;
        node.Lo = Eigen::Vector3f(0.f, 0.f, 0.f);
        node.wo = -ray.direction();
        node.pos_out = ray.origin();
        node.inter = inter;         /* 返回相交的信息，后续的分析要用 */
        return {std::move(node)};
    }

    /* 与发光体相交 */
    if (inter.obj()->material().is_emission()) {
        PathNode node;
        node.Lo = inter.obj()->material().emission();
        node.wo = -ray.direction();
        node.pos_out = ray.origin();
        node.inter = inter;
        return {std::move(node)};
    }

    /* 和不发光的物体相交 */
    std::deque<PathNode> path{};
    cast_ray_recursive(ray, inter, path);
    return path;
}

void
RTRender::write_to_file(const std::vector<PixelType> &buffer, const char *file_path, int width, int height) {
    assert(height > 0 && width > 0);

    /* 打开文件 */
    FILE *fp = fopen(file_path, "wb");
    fprintf(fp, "P6\n%d %d\n255\n", width, height);

    /* 写入文件 */
    for (unsigned row = 0; row < height; ++row) {
        for (unsigned col = 0; col < width; ++col) {
            fwrite(&buffer[row * width + col][0], sizeof(unsigned char), 3, fp);
        }
    }

    fclose(fp);
}

std::vector<RTRender::RenderPixelTask> RTRender::_prepare_render_task(const std::shared_ptr<Scene> &scene) {
    std::vector<RenderPixelTask> task_list;
    task_list.reserve(scene->screen_height() * scene->screen_width());

    /* 设 view 平面位于摄像机前 1.0 处，根据 fov 和 aspect 计算出 view 平面的长和宽 */
    float view_height = 2.f * std::tan(scene->fov() / 2.f / 180.f * M_PI);
    float view_width = view_height / scene->screen_height() * scene->screen_height();

    for (int row = 0; row < scene->screen_height(); ++row) {
        for (int col = 0; col < scene->screen_width(); ++col) {
            /* 像素点在摄像机坐标系中的 x 坐标和 y 坐标 */
            float view_x = ((col + 0.5f) / scene->screen_width() - 0.5f) * view_width;
            float view_y = (0.5f - (row + 0.5f) / scene->screen_height()) * view_height;

            /* 像素点在 global 坐标系中的位置 */
            Eigen::Vector4f dir_global = scene->view_to_global({view_x, view_y, -1.f, 0.f});

            Ray ray(scene->camera_pos(), dir_global.head(3));
            task_list.push_back(RenderPixelTask{col, row, ray});
        }
    }

    return task_list;
}

void RTRender::render_multi_thread(const std::string &db_path, int worker_cnt, int worker_buffer_size,
                                   int worker_sleep_ms,
                                   int master_process_interval) {

    /* 创建任务列表以及保护任务列表的互斥量 */
    std::mutex task_mtx;
    auto task_list = _prepare_render_task(_scene);
    size_t total_task_cnt = task_list.size();

    /* 创建结果列表以及保护结果列表的互斥量 */
    std::mutex res_mtx;
    std::vector<std::shared_ptr<RenderPixelResult>> res_list;

    /* 初始化 worker */
    std::vector<Worker<RenderPixelTask, std::shared_ptr<RenderPixelResult>>>
            workers(worker_cnt, Worker<RenderPixelTask, std::shared_ptr<RenderPixelResult>>(
            task_list, task_mtx, res_list, res_mtx, job_render_one_pixel,
            worker_buffer_size, worker_sleep_ms));

    /* 让 worker 运行 */
    for (auto &worker : workers)
        worker.start();
    fmt::print("\n");

    /* 连接到数据库，清空旧数据 */
    DB::init_db(db_path);
    sqlite3_exec(DB::db, "DELETE FROM node", nullptr, nullptr, nullptr);
    sqlite3_exec(DB::db, "DELETE FROM path", nullptr, nullptr, nullptr);

    /* 直到所有 result 都被处理过，才会停止 */
    size_t processed_res_cnt = 0;           /* 已经处理过的结果数量 */
    while (processed_res_cnt < total_task_cnt) {

        /* 是否有 result 需要处理，如果没有，就 sleep */
        if (res_list.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(master_process_interval));
            continue;
        }

        auto start = std::chrono::system_clock::now();
        /* 将 res_list 的数据转移到 res_buffer 中 */
        std::vector<std::shared_ptr<RenderPixelResult>> res_buffer;
        {
            std::lock_guard<std::mutex> lck(res_mtx);
            res_buffer.insert(res_buffer.end(), res_list.begin(), res_list.end());
            res_list.clear();
        }

        DB::transaction_begin();
        for (const auto &res : res_buffer) {
            /* 将光路信息写入 framebuffer */
            process_render_result(res);

            /* 将光路信息写入数据库 */
            insert_pixel_ray(DB::db, *res);
        }
        DB::transaction_commit();
        processed_res_cnt += res_buffer.size();

        /* 这一轮的处理时间没有达到设定的时间，就睡过去 */
        auto stop = std::chrono::system_clock::now();
        auto time_delta = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
        if (time_delta < master_process_interval)
            std::this_thread::sleep_for(std::chrono::milliseconds(master_process_interval - time_delta));

        /* 更新进度 */
        fmt::print("\rtasks: ({} / {}), result: ({}/{})",
                   task_list.size(), total_task_cnt,
                   processed_res_cnt, total_task_cnt);
        fflush(stdout);
    }
    fmt::print("\n");

    /* 关闭数据库 */
    DB::close_db();

    /* 关闭所有的 worker */
    for (auto &worker : workers)
        worker.stop();
}

void RTRender::render_single_thread(const std::string &db_path) {
    std::vector<RenderPixelTask> render_tasks = _prepare_render_task(_scene);

    /* 连接到数据库，并清空数据 */
    DB::init_db(db_path);
    sqlite3_exec(DB::db, "DELETE FROM node", nullptr, nullptr, nullptr);
    sqlite3_exec(DB::db, "DELETE FROM path", nullptr, nullptr, nullptr);

    fmt::print("tasks: (0 / 0)");
    for (int i = 0; i < render_tasks.size(); ++i) {
        /* 计算一个像素的光路信息 */
        auto res = job_render_one_pixel(render_tasks[i]);

        /* 处理光路信息 */
        process_render_result(res);

        /* 将光路信息写入数据库 */
        insert_pixel_ray(DB::db, *res);

        /* 更新进度 */
        fmt::print("\rtasks: ({} / {})", i, render_tasks.size());
    }

    DB::close_db();
}
