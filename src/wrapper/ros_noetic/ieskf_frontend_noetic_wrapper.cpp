#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
#include "ieskf_slam/CloudWithPose.h"
namespace ROSNoetic
{
    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh)
    {
        std::string config_file_name,lidar_topic,imu_topic;
        nh.param<std::string>("wrapper/config_file_name",config_file_name,"");
        nh.param<std::string>("wrapper/lidar_topic",lidar_topic,"/lidar");
        nh.param<std::string>("wrapper/imu_topic",imu_topic,"/imu");
        std::string time_unit_str;
        nh.param<std::string>("wrapper/time_unit", time_unit_str,"1e-3");
        //using nh.param<double> can not pass parameter, need to figure out why.
        front_end_ptr = std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR+config_file_name,"front_end");
        time_unit = stod(time_unit_str);
        
        // 发布者和订阅者
        cloud_subscriber = nh.subscribe(lidar_topic,1000,&IESKFFrontEndWrapper::lidarCloudMsgCallBack,this);
        imu_subscriber = nh.subscribe(imu_topic,1000,&IESKFFrontEndWrapper::imuMsgCallBack,this);
        // 读取雷达类型
        int lidar_type = 0;
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        if (lidar_type == AVIA) {
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        } else if (lidar_type == VELO) {
            lidar_process_ptr = std::make_shared<VelodyneProcess>();

        } else {
            std::cout << "unsupport lidar type" << std::endl;
            exit(100);
        }
        curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud",100);
        path_pub = nh.advertise<nav_msgs::Path>("path",100);
        local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_map",100);
        cloud_pose_pub = nh.advertise<ieskf_slam::CloudWithPose>("cloud_with_pose", 100);
        run();

    }
    
    IESKFFrontEndWrapper::~IESKFFrontEndWrapper()
    {
    }
    void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg){
        IESKFSlam::PointCloud cloud;
        lidar_process_ptr->process(*msg,cloud,time_unit);
        front_end_ptr->addPointCloud(cloud);
    }
    
    void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg){
        IESKFSlam::IMU imu;
        imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
        imu.acceleration = {msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z};
        imu.gyroscope = {msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z};
        front_end_ptr->addImu(imu);
    }
    void IESKFFrontEndWrapper::run(){
        ros::Rate rate(500);            
       
        while(ros::ok()){ 
            rate.sleep();
            ros::spinOnce();

            if(front_end_ptr->track()){
                publishMsg();
            }
        }
    }
    void IESKFFrontEndWrapper::publishMsg(){
        
        static nav_msgs::Path path;
        auto X =  front_end_ptr->readState();
        path.header.frame_id="map";
        geometry_msgs::PoseStamped psd;
        psd.pose.position.x = X.position.x();
        psd.pose.position.y = X.position.y();
        psd.pose.position.z = X.position.z();
        path.poses.push_back(psd);
        path_pub.publish(path);
        IESKFSlam::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();
        pcl:: transformPointCloud(cloud,cloud,IESKFSlam::compositeTransform(X.rotation,X.position).cast<float>());
        // auto cloud =front_end_ptr->readCurrentPointCloud();
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud,msg);
        msg.header.frame_id = "map";
        curr_cloud_pub.publish(msg);

        cloud = front_end_ptr->readCurrentLocalMap();
        pcl::toROSMsg(cloud,msg);
        msg.header.frame_id = "map";
        local_map_pub.publish(msg);
            ieskf_slam::CloudWithPose cloud_with_pose_msg;

    cloud = front_end_ptr->readCurrentFullPointCloud();
    pcl::toROSMsg(cloud, cloud_with_pose_msg.point_cloud);
    cloud_with_pose_msg.pose.position = psd.pose.position;
    cloud_with_pose_msg.pose.orientation.x = X.rotation.x();
    cloud_with_pose_msg.pose.orientation.y = X.rotation.y();
    cloud_with_pose_msg.pose.orientation.z = X.rotation.z();
    cloud_with_pose_msg.pose.orientation.w = X.rotation.w();
    cloud_pose_pub.publish(cloud_with_pose_msg);
              
    }
} // namespace ROSNoetic