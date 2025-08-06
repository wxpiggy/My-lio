#pragma once
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/type/pose.h"
#include "ieskf_slam/type/measure_group.h"
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/modules/map/rect_map_manager.h"
#include "ieskf_slam/modules/frontbackPropagate/frontbackPropagate.h"
#include "ieskf_slam/modules/frontend/lio_zh_model.h"
#include "ieskf_slam/modules/invkf/invkf.h"
#include <fstream>
namespace IESKFSlam
{
    class FrontEnd: private ModuleBase
    {
    public:
        using Ptr = std::shared_ptr<FrontEnd>;//前端里程计的别名
    private:
        std::deque<IMU> imu_deque;
        std::deque<PointCloud> pointcloud_deque;
        std::deque<Pose> pose_deque; 
        PCLPointCloud current_pointcloud;
        std::shared_ptr<IESKF> ieskf_ptr;
        std::shared_ptr<INVKF> invkf_ptr;
        std::shared_ptr<RectMapManager> map_ptr;
        std::shared_ptr<FrontbackPropagate> fbpropagate_ptr;
        LIOZHModel::Ptr lio_zh_model_ptr;
        LIOZHModelINV::Ptr lio_zh_model_inv_ptr;
        PCLPointCloudPtr filter_point_cloud_ptr, full_point_cloud_ptr;
        VoxelFilter voxel_filter;
        bool imu_inited = false;
        double imu_scale = 1;
        Eigen::Quaterniond extrin_r;
        Eigen::Vector3d extrin_t;
        int use_inv;
        bool trajectory_save;
        std::string trajectory_save_file_name;
        std::fstream trajectory_save_file;
        
    public:
        FrontEnd(const std::string &config_file_path,const std::string & prefix );
        ~FrontEnd();
        // 需要向前端传入imu和点云数据
        void addImu(const IMU&imu);
        void addPointCloud(const PointCloud&pointcloud);
        //void addPose(const Pose&pose);
        // 跟踪
        bool track();
        // 点云读取
        const PCLPointCloud &readCurrentFullPointCloud();

        const PCLPointCloud &readCurrentPointCloud();
        const PCLPointCloud &readCurrentLocalMap();
        bool syncMeasureGroup(MeasureGroup &mg);
        void initState(MeasureGroup &mg);
        IESKF::State18 readState();
        IESKF::State18 readState_inv();
    };
} // namespace IESKFSlam
