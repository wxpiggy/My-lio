#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
#include "ieskf_slam/globaldefine.h"
#include "ieskf_slam/tools/timer.h"

namespace ROSNoetic {

IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh) {
    nh.param<std::string>("wrapper/mode", mode_, "offline");
    nh.param<std::string>("wrapper/bag_path", bag_path_, "");
    nh.param<double>("wrapper/speed_factor", speed_factor_, 1.0);
    nh.param<std::string>("wrapper/visualization_mode", visualization_mode_, "rviz");
    nh.param<bool>("wrapper/save_pcd", save_pcd, false);
    // std::string config_file_name, lidar_topic, imu_topic;
    nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
    nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
    nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");
    std::string time_unit_str;
    nh.param<std::string>("wrapper/time_unit", time_unit_str, "1e-3");
    time_unit = std::stod(time_unit_str);

    front_end_ptr = std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");

    int lidar_type = 0;
    nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
    if (lidar_type == AVIA) {
        lidar_process_ptr = std::make_shared<AVIAProcess>();
    } else if (lidar_type == VELO) {
        lidar_process_ptr = std::make_shared<VelodyneProcess>();
    } else {
        ROS_ERROR("Unsupported lidar type");
        exit(100);
    }

    // curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
    // path_pub = nh.advertise<nav_msgs::Path>("path", 100);
    // local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map", 100);

    if (mode_ == "realtime") {
        cloud_subscriber = nh.subscribe(lidar_topic, 1000, &IESKFFrontEndWrapper::lidarCloudMsgCallBack, this);
        imu_subscriber = nh.subscribe(imu_topic, 1000, &IESKFFrontEndWrapper::imuMsgCallBack, this);
    }
    if(visualization_mode_ == "pangolin"){
        initializePangolinVisualization();
    }
        
    run();
}

IESKFFrontEndWrapper::~IESKFFrontEndWrapper() {
    if (pangolin_viz_) {
        pangolin_viz_->stop();
    }

    
}

void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg) {
    IESKFSlam::PointCloud cloud;
    lidar_process_ptr->process(*msg, cloud, time_unit);
    front_end_ptr->addPointCloud(cloud);
}

void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg) {
    IESKFSlam::IMU imu;
    imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
    imu.acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    imu.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    front_end_ptr->addImu(imu);
}

void IESKFFrontEndWrapper::publishMsg() {
    auto X = front_end_ptr->readState();
    IESKFSlam::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();
    pcl::transformPointCloud(cloud, cloud,
                            IESKFSlam::compositeTransform(X.rotation, X.position).cast<float>());

    if (visualization_mode_ == "rviz") {
        static nav_msgs::Path path;
        path.header.frame_id = "map";

        geometry_msgs::PoseStamped psd;
        psd.header.frame_id = "map";
        psd.header.stamp = ros::Time::now();
        psd.pose.position.x = X.position.x();
        psd.pose.position.y = X.position.y();
        psd.pose.position.z = X.position.z();
        psd.pose.orientation.x = X.rotation.x();
        psd.pose.orientation.y = X.rotation.y();
        psd.pose.orientation.z = X.rotation.z();
        psd.pose.orientation.w = X.rotation.w();
        path.poses.push_back(psd);

        path_pub.publish(path);
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = ros::Time::now();
        curr_cloud_pub.publish(cloud_msg);

        IESKFSlam::PCLPointCloud local_map = front_end_ptr->readCurrentLocalMap();
        pcl::toROSMsg(local_map, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = ros::Time::now();
        local_map_pub.publish(cloud_msg);
    }
    else if (visualization_mode_ == "pangolin") {
        publishMsgPangolin(X, cloud);
    }
}

void IESKFFrontEndWrapper::publishMsgPangolin(const IESKFSlam::IESKF::State18 &state,
                                              const IESKFSlam::PCLPointCloud &cloud) {
    if (!pangolin_viz_) return;
    
    // 更新位姿
    pangolin_viz_->updatePose(state.position, state.rotation);
    
    // 转换PCL点云格式
    pcl::PointCloud<pcl::PointXYZI> xyz_cloud;
    pcl::copyPointCloud(cloud, xyz_cloud);
    pangolin_viz_->updateCurrentCloud(xyz_cloud);
    
    // 更新局部地图
    IESKFSlam::PCLPointCloud local_map = front_end_ptr->readCurrentLocalMap();
    pcl::PointCloud<pcl::PointXYZI> xyz_local_map;
    pcl::copyPointCloud(local_map, xyz_local_map);
    pangolin_viz_->updateLocalMap(xyz_local_map);
}
void IESKFFrontEndWrapper::initializePangolinVisualization() {
    if (visualization_mode_ == "pangolin") {
        pangolin_viz_ = std::make_unique<PangolinVisualizer>();
        pangolin_viz_->start();
    }
}
void IESKFFrontEndWrapper::playBagToIESKF_Streaming(const std::string &bag_path, double speed_factor) {
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    std::vector<std::string> topics = {imu_topic,lidar_topic};
    
    std::cout << imu_topic << std::endl;
    std::cout << lidar_topic << std::endl;

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    ros::Time prev_time;
    bool first = true;

    ROS_INFO("Start playing bag with speed_factor=%f", speed_factor);

    int count = 0;
    for (const rosbag::MessageInstance &msg : view) {

        if (!ros::ok()) {
            ROS_WARN("ros::ok() is false, exiting bag playback");
            break;
        }
        ros::Time curr_time = msg.getTime();

if (first) {
    prev_time = curr_time;
    first = false;
} else {
    double dt = (curr_time - prev_time).toSec();
    prev_time = curr_time;

    if (dt > 0 && speed_factor > 0 && speed_factor != 1) {
        // 1倍速不休眠，其他倍速根据倍数调整休眠时间
        double sleep_time = dt / speed_factor;
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
}

        count++;
        // ROS_INFO("Processing message #%d at time %f", count, curr_time.toSec());

        if (msg.isType<sensor_msgs::Imu>()) {
            auto imu_msg = msg.instantiate<sensor_msgs::Imu>();
            if (imu_msg) {
                IESKFSlam::IMU imu;
                imu.time_stamp.fromNsec(imu_msg->header.stamp.toNSec());
                imu.acceleration = {imu_msg->linear_acceleration.x,
                                    imu_msg->linear_acceleration.y,
                                    imu_msg->linear_acceleration.z};
                imu.gyroscope = {imu_msg->angular_velocity.x,
                                 imu_msg->angular_velocity.y,
                                 imu_msg->angular_velocity.z};
                front_end_ptr->addImu(imu);
            }
        } else if (msg.isType<sensor_msgs::PointCloud2>()) {
            auto cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_msg) {
                IESKFSlam::PointCloud cloud;
                lidar_process_ptr->process(*cloud_msg, cloud, time_unit);
                front_end_ptr->addPointCloud(cloud);

                bool track_result = false;
                Timer([&](){
                    track_result = front_end_ptr->track();
                }, "完整流程");

                if (track_result) {
                    publishMsg();
                }

                        }
        }

    }
    savePCD();
    ROS_INFO("Finished playing bag with %d messages", count);

    bag.close();
}

void IESKFFrontEndWrapper::run() {
    if (mode_ == "offline") {
        ROS_INFO_STREAM("Running in offline mode, playing bag: " << bag_path_ << " at speed factor: " << speed_factor_);
       
        playBagToIESKF_Streaming(bag_path_, speed_factor_);
    } else {
        ROS_INFO("Running in realtime mode (ROS subscription).");
        ros::Rate rate(500);
        while (ros::ok()) {
            rate.sleep();
            ros::spinOnce();
            if (front_end_ptr->track()) {
                publishMsg();
            }
        }
    }
}
void IESKFFrontEndWrapper::savePCD(){
    auto &global_map = front_end_ptr->readGlobalMap();
    if (!global_map.empty()) {
        pcl::io::savePCDFileBinary(RESULT_DIR+"global_map.pcd", global_map);
        ROS_INFO("Saved global map to global_map.pcd");
    } else {
        ROS_WARN("Global map is empty, nothing to save.");
    }
}
}  // namespace ROSNoetic
