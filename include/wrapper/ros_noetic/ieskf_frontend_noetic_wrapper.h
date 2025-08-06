#pragma once
#include "ieskf_slam/modules/frontend/frontend.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "wrapper/ros_noetic/lidar_process/avia_process.h"
#include "wrapper/ros_noetic/lidar_process/velodyne_process.h"
#include "ieskf_slam/globaldefine.h"

namespace ROSNoetic {
    enum LIDAR_TYPE {
        AVIA = 0, VELO = 1

    };

    class IESKFFrontEndWrapper {
    private:
        IESKFSlam::FrontEnd::Ptr front_end_ptr;
        ros::Subscriber cloud_subscriber;
        ros::Subscriber imu_subscriber;
        ros::Subscriber odometry_subscriber;
        ros::Publisher curr_cloud_pub;
        ros::Publisher path_pub;
        ros::Publisher local_map_pub;
        ros::Publisher cloud_pose_pub;
        std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr;

        // cur status
        IESKFSlam::PCLPointCloud curr_cloud;
        Eigen::Quaterniond curr_q;
        Eigen::Vector3d curr_t;
        double time_unit=0.0;
        void lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg);
        void imuMsgCallBack(const sensor_msgs::ImuPtr &msg);
        //void odometryMsgCallBack(const nav_msgs::OdometryPtr &msg);
        void run();
        void publishMsg();

    public:
        IESKFFrontEndWrapper(ros::NodeHandle &nh);
        ~IESKFFrontEndWrapper();
    };
}  // namespace ROSNoetic