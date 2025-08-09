#ifndef IESKF_SLAM_TOOLS_PANGOLIN_VISUALIZER_H_
#define IESKF_SLAM_TOOLS_PANGOLIN_VISUALIZER_H_

#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

class PangolinVisualizer {
public:
    // 内部Pose结构体
    struct Pose {
        Eigen::Vector3d position;
        Eigen::Quaterniond rotation;

        Pose();
        Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot);
    };

    PangolinVisualizer();
    ~PangolinVisualizer();

    // 启动与停止渲染线程
    void start();
    void stop();

    // 更新数据接口
    void updatePose(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation);
    void updateCurrentCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    void updateLocalMap(const pcl::PointCloud<pcl::PointXYZI>& map);

private:
    // 渲染主循环
    void renderLoop();

    // 初始化窗口及UI控件
    void initializePangolin();

    // 各类绘制函数
    void drawTrajectory();
    void drawPointCloudColorMode(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    void drawCurrentPose();
    void drawCoordinateFrame();
    void BuildIntensityTable();

private:
    // 线程和同步
    std::thread render_thread_;
    bool should_stop_;
    std::mutex data_mutex_;

    // 相机和显示面板
    pangolin::OpenGlRenderState s_cam_;
    pangolin::View d_cam_;

    std::vector<Eigen::Vector4d> intensity_color_table_pcl_;

    // UI控件
    std::unique_ptr<pangolin::Var<float>> menu_point_size_;
    std::unique_ptr<pangolin::Var<int>> menu_trajectory_length_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_trajectory_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_current_cloud_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_local_map_;
    std::unique_ptr<pangolin::Var<bool>> menu_background_dark_;
    std::unique_ptr<pangolin::Var<bool>> menu_point_color_mode_;
    std::unique_ptr<pangolin::Var<bool>> menu_auto_follow_;

    // 点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_;

    // 位姿轨迹数据
    std::deque<Pose> trajectory_;
    Pose current_pose_;
Eigen::Vector4d IntensityToRgbPCL(double val) const {
    const int table_size = static_cast<int>(intensity_color_table_pcl_.size());
    double idx_f = val * (table_size - 1);
    int idx0 = static_cast<int>(std::floor(idx_f)) % table_size;
    int idx1 = (idx0 + 1) % table_size;
    double t = idx_f - std::floor(idx_f);

    const Eigen::Vector4d& c0 = intensity_color_table_pcl_[idx0];
    const Eigen::Vector4d& c1 = intensity_color_table_pcl_[idx1];
    return c0 * (1.0 - t) + c1 * t;
}
    // 静态颜色常量
    static constexpr float trajectory_color_[3] = {0.0f, 0.8f, 0.2f};
    static constexpr float current_cloud_color_[3] = {0.0f, 0.6f, 1.0f};
    static constexpr float local_map_color_[3] = {1.0f, 0.3f, 0.3f};
    static constexpr float pose_color_[3] = {0.9f, 0.9f, 0.9f};
};

 // namespace ieskf_slam

#endif  // IESKF_SLAM_TOOLS_PANGOLIN_VISUALIZER_H_