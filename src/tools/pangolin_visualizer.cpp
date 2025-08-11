#include "ieskf_slam/tools/pangolin_visualizer.h"
#include <Eigen/src/Core/Matrix.h>
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

Eigen::Matrix4d getFollowCamPose(const PangolinVisualizer::Pose& robot_pose) {
    // 机器人位姿
    Eigen::Vector3d pos = robot_pose.position;
    Eigen::Matrix3d rot = robot_pose.rotation.toRotationMatrix();

    // 定义相机相对机器人坐标系的位置，例如在机器人后方2米，高度1米
    Eigen::Vector3d cam_offset(-2.0, 0.0, 1.0);

    // 计算相机世界坐标 = 机器人位置 + 旋转 * 偏移
    Eigen::Vector3d cam_pos = pos + rot * cam_offset;

    // 相机目标点就是机器人当前的位置
    Eigen::Vector3d target = pos;

    // 定义相机的上方向（一般用世界坐标系的Z轴）
    Eigen::Vector3d up(0, 0, 1);

    // 使用类似 gluLookAt 实现，构造4x4视图矩阵
    Eigen::Vector3d z_axis = (cam_pos - target).normalized();  // 视线方向
    Eigen::Vector3d x_axis = up.cross(z_axis).normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();

    view(0,0) = x_axis.x(); view(0,1) = x_axis.y(); view(0,2) = x_axis.z();
    view(1,0) = y_axis.x(); view(1,1) = y_axis.y(); view(1,2) = y_axis.z();
    view(2,0) = z_axis.x(); view(2,1) = z_axis.y(); view(2,2) = z_axis.z();

    view(0,3) = -x_axis.dot(cam_pos);
    view(1,3) = -y_axis.dot(cam_pos);
    view(2,3) = -z_axis.dot(cam_pos);

    return view;
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
void PangolinVisualizer::BuildIntensityTable() {
intensity_color_table_pcl_.clear();
    intensity_color_table_pcl_.reserve(256);
    
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f;
        
        float r, g, b;
        
        // Jet colormap算法
        if (t < 0.125f) {
            r = 0.0f;
            g = 0.0f;
            b = 0.5f + t * 4.0f;
        } else if (t < 0.375f) {
            r = 0.0f;
            g = (t - 0.125f) * 4.0f;
            b = 1.0f;
        } else if (t < 0.625f) {
            r = (t - 0.375f) * 4.0f;
            g = 1.0f;
            b = 1.0f - (t - 0.375f) * 4.0f;
        } else if (t < 0.875f) {
            r = 1.0f;
            g = 1.0f - (t - 0.625f) * 4.0f;
            b = 0.0f;
        } else {
            r = 1.0f - (t - 0.875f) * 4.0f;
            g = 0.0f;
            b = 0.0f;
        }
        
        // 限制范围并降低饱和度
        r = std::max(0.0f, std::min(1.0f, r)) * 0.9f;
        g = std::max(0.0f, std::min(1.0f, g)) * 0.9f;
        b = std::max(0.0f, std::min(1.0f, b)) * 0.9f;
        
        intensity_color_table_pcl_.emplace_back(r, g, b, 0.8f);
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
         pangolin::ProjectionMatrix(1920, 1080, 960, 960, 960, 540, 0.1, 5000),
            pangolin::ModelViewLookAt(
        0, 0, 250,   // 摄像机位置：X=0, Y=0, Z=100，很高的高度
        0, 0, 0,     // 观察目标点：原点
        1, 0, 0      // 摄像机“上”方向，这里用X轴方向，保证摄像机是“正顶视”（从上往下看）
    )
    );

    // 左侧200像素菜单面板
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(300));

    // 右侧3D视图，自适应窗口宽高变化
    d_cam_ = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1200.0f / 800.0f)
        .SetHandler(new pangolin::Handler3D(s_cam_));

    // UI控件
    menu_point_size_ = std::make_unique<pangolin::Var<float>>("menu.Point Size", 1.0f, 1.0f, 10.0f);
    menu_trajectory_length_ = std::make_unique<pangolin::Var<int>>("menu.Trajectory Length", 100000, 100, 100000);
    menu_show_trajectory_ = std::make_unique<pangolin::Var<bool>>("menu.Show Trajectory", true, true);
    menu_show_current_cloud_ = std::make_unique<pangolin::Var<bool>>("menu.Show Current Cloud", true, true);
    menu_show_local_map_ = std::make_unique<pangolin::Var<bool>>("menu.Show Local Map", true, true);
    menu_background_dark_ = std::make_unique<pangolin::Var<bool>>("menu.Dark Background", false, false);
    menu_auto_follow_ = std::make_unique<pangolin::Var<bool>>("menu.Auto Follow", true, true);
    
    // 新增点云颜色模式切换 0 = Z轴, 1 = Intensity
    menu_point_color_mode_ = std::make_unique<pangolin::Var<bool>>("menu.Point Color Mode", true,false);
    BuildIntensityTable();
}
pangolin::OpenGlMatrix PoseToOpenGlMatrix(const PangolinVisualizer::Pose& pose) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3,3>(0,0) = pose.rotation.toRotationMatrix();
    mat.block<3,1>(0,3) = pose.position;

    pangolin::OpenGlMatrix ogl_mat;
    // Pangolin是列主序的4x4矩阵
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            ogl_mat.m[c * 4 + r] = static_cast<float>(mat(r,c));
        }
    }
    return ogl_mat;
}
// 渲染循环
void PangolinVisualizer::renderLoop() {
    initializePangolin();

    // 初始化自动跟随菜单

    // 用户交互状态和计时器
    bool user_interacting_ = false;
    auto last_interaction_time = std::chrono::steady_clock::now();

    // 你需要在鼠标事件里（可用Pangolin事件回调或poll）设置 user_interacting_ = true，刷新 last_interaction_time
    // 这里简单示范一个超时机制，若1秒无交互，重置为 false
    auto interaction_timeout = std::chrono::seconds(1);

    while (!pangolin::ShouldQuit() && !should_stop_) {
        // 处理用户交互检测（此处无事件接口，需要你自行添加事件监听）
        // 简单逻辑示例，建议在你鼠标回调中调用此函数更新状态
        auto now = std::chrono::steady_clock::now();
        if (user_interacting_ && now - last_interaction_time > interaction_timeout) {
            user_interacting_ = false;
        }

        if (*menu_background_dark_) {
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        } else {
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 动态设置视角
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            if (*menu_auto_follow_ && !user_interacting_ && !trajectory_.empty()) {
                s_cam_.Follow(PoseToOpenGlMatrix(current_pose_));
            }
            // 否则用户自由操作，保留现有视角，不强制修改

            d_cam_.Activate(s_cam_);

            auto bounds = d_cam_.GetBounds();
            int w = static_cast<int>(bounds.w);
            int h = static_cast<int>(bounds.h);
            glViewport(static_cast<int>(bounds.l), static_cast<int>(bounds.b), w, h);

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

    glLineWidth(6.0f);  // 加粗轨迹线宽，默认是3.0f，你可以调大
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f); // 红色，且不透明
    glBegin(GL_LINE_STRIP);
    for (const auto& pose : trajectory_) {
        glVertex3f(pose.position.x(), pose.position.y(), pose.position.z());
    }
    glEnd();

    glPointSize(8.0f); // 点大小也调大些
    glBegin(GL_POINTS);
    for (const auto& pose : trajectory_) {
        glVertex3f(pose.position.x(), pose.position.y(), pose.position.z());
    }
    glEnd();
}



void PangolinVisualizer::drawPointCloudColorMode(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (cloud.empty()) return;
    
    // 启用混合模式使透明度生效
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(*menu_point_size_);
    
    float min_intensity = std::numeric_limits<float>::max();
    float max_intensity = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    
    for (const auto& p : cloud.points) {
        if (std::isfinite(p.intensity)) {
            min_intensity = std::min(min_intensity, p.intensity);
            max_intensity = std::max(max_intensity, p.intensity);
        }
        if (std::isfinite(p.z)) {
            min_z = std::min(min_z, p.z);
            max_z = std::max(max_z, p.z);
        }
    }
    
    float intensity_range = max_intensity - min_intensity;
    if (intensity_range < 1e-6) intensity_range = 1.0f;
    float z_range = max_z - min_z;
    if (z_range < 1e-6) z_range = 1.0f;
    
    glBegin(GL_POINTS);
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        
        float norm_val = 0.f;
        if (*menu_point_color_mode_ == false) {
            // Z轴模式 - 使用归一化的z值，更平滑的渐变
            norm_val = (p.z - min_z) / z_range;
            // 可选：增强对比度
            norm_val = std::pow(norm_val, 0.8f); // gamma校正，让中间值更亮
        } else if (*menu_point_color_mode_ == true) {
            // Intensity模式 - 使用归一化的强度值
            norm_val = (p.intensity - min_intensity) / intensity_range;
            // 可选：根据数据特点调整映射策略
            
            // 策略1: 线性映射（默认，最平滑）
            // norm_val = norm_val; // 已经是归一化的
            
            // 策略2: 对数映射（适合强度差异很大的数据）
            // norm_val = std::log(1.0f + norm_val * 9.0f) / std::log(10.0f);
            
            // 策略3: 平方根映射（增强低强度细节）
            // norm_val = std::sqrt(norm_val);
            
            // 策略4: S型曲线（增强中等强度对比度）
            // norm_val = norm_val * norm_val * (3.0f - 2.0f * norm_val);
        }
        
        // 确保值在[0,1]范围内
        norm_val = std::max(0.0f, std::min(1.0f, norm_val));
        
        Eigen::Vector4d c = IntensityToRgbPCL(norm_val);
        glColor4f(c.x(), c.y(), c.z(), c.w());
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
    
    glDisable(GL_POINT_SPRITE);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_BLEND);
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
