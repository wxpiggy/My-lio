#include "ieskf_slam/tools/pangolin_visualizer.h"
#include <pangolin/pangolin.h>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <cmath>

// 静态颜色初始化
constexpr float PangolinVisualizer::trajectory_color_[3];
constexpr float PangolinVisualizer::current_cloud_color_[3];
constexpr float PangolinVisualizer::local_map_color_[3];
constexpr float PangolinVisualizer::pose_color_[3];

// Pose结构体
PangolinVisualizer::Pose::Pose()
    : position(Eigen::Vector3d::Zero()),
      rotation(Eigen::Quaterniond::Identity()) {}

PangolinVisualizer::Pose::Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot)
    : position(pos), rotation(rot) {}

// 构造析构
PangolinVisualizer::PangolinVisualizer()
    : should_stop_(false),
      current_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
      local_map_(new pcl::PointCloud<pcl::PointXYZI>()) {}

PangolinVisualizer::~PangolinVisualizer() {
    stop();
}

// 启动渲染线程
void PangolinVisualizer::start() {
    render_thread_ = std::thread(&PangolinVisualizer::renderLoop, this);
}

// 停止渲染线程
void PangolinVisualizer::stop() {
    should_stop_ = true;
    if (render_thread_.joinable()) {
        render_thread_.join();
    }
}

// 更新位姿
void PangolinVisualizer::updatePose(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_pose_ = Pose(position, rotation);
    trajectory_.push_back(current_pose_);
    if (trajectory_.size() > static_cast<size_t>(*menu_trajectory_length_)) {
        trajectory_.pop_front();
    }
}

// 更新当前点云
void PangolinVisualizer::updateCurrentCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    *current_cloud_ = cloud;
}

// 更新局部地图
void PangolinVisualizer::updateLocalMap(const pcl::PointCloud<pcl::PointXYZI>& map) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    *local_map_ = map;
}

// 初始化Pangolin窗口和UI
void PangolinVisualizer::initializePangolin() {
    pangolin::CreateWindowAndBind("IESKF SLAM Visualization", 1920, 1080);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    s_cam_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1200, 800, 600, 600, 600, 400, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // 左侧200像素菜单面板
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(200));

    // 右侧3D视图，自适应窗口宽高变化
    d_cam_ = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1200.0f / 800.0f)
        .SetHandler(new pangolin::Handler3D(s_cam_));

    // UI控件
    menu_point_size_ = std::make_unique<pangolin::Var<float>>("menu.Point Size", 1.0f, 1.0f, 10.0f);
    menu_trajectory_length_ = std::make_unique<pangolin::Var<int>>("menu.Trajectory Length", 10000, 100, 10000);
    menu_show_trajectory_ = std::make_unique<pangolin::Var<bool>>("menu.Show Trajectory", true, true);
    menu_show_current_cloud_ = std::make_unique<pangolin::Var<bool>>("menu.Show Current Cloud", true, true);
    menu_show_local_map_ = std::make_unique<pangolin::Var<bool>>("menu.Show Local Map", true, true);
    menu_background_dark_ = std::make_unique<pangolin::Var<bool>>("menu.Dark Background", true, true);
    
    // 新增点云颜色模式切换 0 = Z轴, 1 = Intensity
    menu_point_color_mode_ = std::make_unique<pangolin::Var<int>>("menu.Point Color Mode", 0, 0, 1);
}

// 渲染循环
void PangolinVisualizer::renderLoop() {
    initializePangolin();

    while (!pangolin::ShouldQuit() && !should_stop_) {
        if (*menu_background_dark_) {
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        } else {
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam_.Activate(s_cam_);

        auto bounds = d_cam_.GetBounds();
        int w = static_cast<int>(bounds.w);
        int h = static_cast<int>(bounds.h);
        glViewport(static_cast<int>(bounds.l), static_cast<int>(bounds.b), w, h);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (*menu_show_trajectory_) drawTrajectory();
            if (*menu_show_current_cloud_ && !current_cloud_->empty()) drawPointCloudColorMode(*current_cloud_);
            if (*menu_show_local_map_ && !local_map_->empty()) drawPointCloudColorMode(*local_map_);
            drawCurrentPose();
            drawCoordinateFrame();
        }

        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    pangolin::DestroyWindow("IESKF SLAM Visualization");
}

// 绘制轨迹
void PangolinVisualizer::drawTrajectory() {
    if (trajectory_.size() < 2) return;

    glLineWidth(3.0f);
    glBegin(GL_LINE_STRIP);
    float alpha_step = 1.0f / trajectory_.size();
    float alpha = 0.3f;
    for (const auto& pose : trajectory_) {
        glColor4f(trajectory_color_[0], trajectory_color_[1], trajectory_color_[2], alpha);
        glVertex3f(pose.position.x(), pose.position.y(), pose.position.z());
        alpha = std::min(1.0f, alpha + alpha_step);
    }
    glEnd();

    glPointSize(4.0f);
    glBegin(GL_POINTS);
    size_t recent_points = std::min(trajectory_.size(), size_t(50));
    auto recent_start = trajectory_.end() - recent_points;

    alpha = 0.5f;
    alpha_step = 0.5f / recent_points;
    for (auto it = recent_start; it != trajectory_.end(); ++it) {
        glColor4f(trajectory_color_[0], trajectory_color_[1], trajectory_color_[2], alpha);
        glVertex3f(it->position.x(), it->position.y(), it->position.z());
        alpha = std::min(1.0f, alpha + alpha_step);
    }
    glEnd();
}

// 根据颜色模式绘制点云（支持Z轴和Intensity）
void PangolinVisualizer::drawPointCloudColorMode(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (cloud.empty()) return;

    glEnable(GL_POINT_SPRITE);
    glEnable(GL_PROGRAM_POINT_SIZE);

    glPointSize(*menu_point_size_);

    // 预先找强度和Z范围，用于归一化颜色映射
    float min_intensity = std::numeric_limits<float>::max();
    float max_intensity = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& p : cloud.points) {
        if (std::isfinite(p.intensity)) {
            if (p.intensity < min_intensity) min_intensity = p.intensity;
            if (p.intensity > max_intensity) max_intensity = p.intensity;
        }
        if (std::isfinite(p.z)) {
            if (p.z < min_z) min_z = p.z;
            if (p.z > max_z) max_z = p.z;
        }
    }

    // 防止除0
    float intensity_range = max_intensity - min_intensity;
    if (intensity_range < 1e-6) intensity_range = 1.0f;
    float z_range = max_z - min_z;
    if (z_range < 1e-6) z_range = 1.0f;

    glBegin(GL_POINTS);
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        float r=1.f, g=1.f, b=1.f;

        if (*menu_point_color_mode_ == 0) {  // 根据Z轴高度上色
            float norm_z = (p.z - min_z) / z_range;
            r = norm_z;
            g = 0.5f * (1.0f - norm_z);
            b = 1.0f - norm_z;
        } else if (*menu_point_color_mode_ == 1) {  // 根据Intensity上色
            float norm_i = (p.intensity - min_intensity) / intensity_range;
            r = norm_i;
            g = 1.0f - norm_i;
            b = 0.0f;
        }

        glColor4f(r, g, b, 0.8f);
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();

    glDisable(GL_POINT_SPRITE);
    glDisable(GL_PROGRAM_POINT_SIZE);
}

// 绘制当前位姿
void PangolinVisualizer::drawCurrentPose() {
    if (trajectory_.empty()) return;

    const Pose& pose = current_pose_;
    Eigen::Vector3d pos = pose.position;
    Eigen::Matrix3d rot = pose.rotation.toRotationMatrix();

    glLineWidth(4.0f);

    // X轴红色
    glColor4f(1.0f, 0.0f, 0.0f, 0.9f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d x_end = pos + rot.col(0) * 0.8;
    glVertex3f(x_end.x(), x_end.y(), x_end.z());
    glEnd();

    // Y轴绿色
    glColor4f(0.0f, 1.0f, 0.0f, 0.9f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d y_end = pos + rot.col(1) * 0.8;
    glVertex3f(y_end.x(), y_end.y(), y_end.z());
    glEnd();

    // Z轴蓝色
    glColor4f(0.0f, 0.0f, 1.0f, 0.9f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d z_end = pos + rot.col(2) * 0.8;
    glVertex3f(z_end.x(), z_end.y(), z_end.z());
    glEnd();

    // 画一个小立方体表示机器人
    glColor4f(pose_color_[0], pose_color_[1], pose_color_[2], 0.9f);
    glPushMatrix();
    glTranslatef(pos.x(), pos.y(), pos.z());

    Eigen::AngleAxisd aa(pose.rotation);
    Eigen::Vector3d axis = aa.axis();
    double angle = aa.angle() * 180.0 / M_PI;
    glRotatef(static_cast<float>(angle), static_cast<float>(axis.x()), static_cast<float>(axis.y()), static_cast<float>(axis.z()));

    pangolin::glDrawColouredCube(-0.15, 0.15);
    glPopMatrix();
}

// 绘制世界坐标系
void PangolinVisualizer::drawCoordinateFrame() {
    glLineWidth(3.0f);

    // X红色
    glColor4f(1.0f, 0.0f, 0.0f, 0.8f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(2, 0, 0);
    glEnd();

    // Y绿色
    glColor4f(0.0f, 1.0f, 0.0f, 0.8f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 2, 0);
    glEnd();

    // Z蓝色
    glColor4f(0.0f, 0.0f, 1.0f, 0.8f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 2);
    glEnd();
}