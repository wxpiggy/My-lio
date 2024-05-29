#include "ieskf_slam/modules/frontend/frontend.h"

#include "pcl/common/transforms.h"

namespace IESKFSlam {
    FrontEnd::FrontEnd(const std::string &config_file_path, const std::string &prefix)
        : ModuleBase(config_file_path, prefix, "Front End Module") {
        float leaf_size;
        readParam("filter_leaf_size", leaf_size, 0.5f);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

        std::vector<double> extrin_v;
        readParam("extrin_r", extrin_v, std::vector<double>());
        extrin_r.setIdentity();
        extrin_t.setZero();
        if (extrin_v.size() == 9) {
            Eigen::Matrix3d extrin_r33;
            extrin_r33 << extrin_v[0], extrin_v[1], extrin_v[2], extrin_v[3], extrin_v[4], extrin_v[5], extrin_v[6],
                extrin_v[7], extrin_v[8];
            extrin_r = extrin_r33;
        } else if (extrin_v.size() == 3) {
            extrin_r.x() = extrin_v[0];
            extrin_r.y() = extrin_v[1];
            extrin_r.z() = extrin_v[2];
            extrin_r.w() = extrin_v[3];
        }
        readParam("extrin_t", extrin_v, std::vector<double>());
        if (extrin_v.size() == 3) {
            extrin_t << extrin_v[0], extrin_v[1], extrin_v[2];
        }
        readParam("use_inv", use_inv, 0);
        
        map_ptr = std::make_shared<RectMapManager>(config_file_path, "map");

        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();
        
       

        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        if (!use_inv) {
            ieskf_ptr = std::make_shared<IESKF>(config_file_path, "ieskf");
            lio_zh_model_ptr = std::make_shared<LIOZHModel>();
            ieskf_ptr->calc_zh_ptr = lio_zh_model_ptr;
            lio_zh_model_ptr->prepare(map_ptr->readKDtree(), filter_point_cloud_ptr, map_ptr->getLocalMap());
        } else {
            invkf_ptr = std::make_shared<INVKF>(config_file_path, "invkf");
            lio_zh_model_inv_ptr = std::make_shared<LIOZHModelINV>();
            invkf_ptr->calc_zh_ptr = lio_zh_model_inv_ptr;
            lio_zh_model_inv_ptr->prepare(map_ptr->readKDtree(), filter_point_cloud_ptr, map_ptr->getLocalMap());
        }

    }

    FrontEnd::~FrontEnd() {}
    void FrontEnd::addImu(const IMU &imu) { imu_deque.push_back(imu); }
    void FrontEnd::addPointCloud(const PointCloud &pointcloud) {
        pointcloud_deque.push_back(pointcloud);
        pcl::transformPointCloud(*pointcloud_deque.back().cloud_ptr, *pointcloud_deque.back().cloud_ptr,
                                 compositeTransform(extrin_r, extrin_t).cast<float>());
        // std::cout << "receive cloud" << std::endl;
    }
    // void FrontEnd::addPose(const Pose& pose) {
    //     pose_deque.push_back(pose);
    //     std::cout << "receive pose" << std::endl;
    // }
    bool FrontEnd::track() {
        MeasureGroup mg;

        if (syncMeasureGroup(mg)) {
            if (!imu_inited) {
                map_ptr->reset();
                map_ptr->addScan(mg.cloud.cloud_ptr, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }

            if (!use_inv) {
                fbpropagate_ptr->propagate(mg, ieskf_ptr);
                voxel_filter.setInputCloud(mg.cloud.cloud_ptr);
                voxel_filter.filter(*filter_point_cloud_ptr);
                ieskf_ptr->update();
                auto x = ieskf_ptr->getX();
                map_ptr->addScan(filter_point_cloud_ptr, x.rotation, x.position);
                return true;
            } else {
                fbpropagate_ptr->propagate(mg, invkf_ptr);

                voxel_filter.setInputCloud(mg.cloud.cloud_ptr);
                voxel_filter.filter(*filter_point_cloud_ptr);
                invkf_ptr->update();
                // auto x = invkf_ptr->getX();
                map_ptr->addScan(filter_point_cloud_ptr, Eigen::Quaterniond(invkf_ptr->getRotation()),
                                 invkf_ptr->getPosition());
                return true;
            }
        }
        return false;

        // if (pose_deque.empty() || pointcloud_deque.empty()) {
        //     return false;
        // }
        // // 寻找同一时刻的点云和位姿

        // while (!pose_deque.empty() &&
        //        pose_deque.front().time_stamp.nsec() < pointcloud_deque.front().time_stamp.nsec()) {
        //     std::cout << "1" << std::endl;
        //     pose_deque.pop_front();
        // }
        // if (pose_deque.empty()) {
        //     return false;
        // }
        // while (!pointcloud_deque.empty() &&
        //        pointcloud_deque.front().time_stamp.nsec() < pose_deque.front().time_stamp.nsec()) {
        //     std::cout << "2" << std::endl;
        //     pointcloud_deque.pop_front();
        // }
        // if (pointcloud_deque.empty()) {
        //     return false;
        // }
        // // 滤波
        // VoxelFilter vf;
        // vf.setLeafSize(0.5, 0.5, 0.5);
        // vf.setInputCloud(pointcloud_deque.front().cloud_ptr);
        // vf.filter(*pointcloud_deque.front().cloud_ptr);

        // Eigen::Matrix4f trans;
        // trans.setIdentity();
        // trans.block<3, 3>(0, 0) = pose_deque.front().rotation.toRotationMatrix().cast<float>();
        // trans.block<3, 1>(0, 3) = pose_deque.front().position.cast<float>();
        // pcl::transformPointCloud(*pointcloud_deque.front().cloud_ptr, current_pointcloud, trans);

        // pointcloud_deque.pop_front();
        // pose_deque.pop_front();
        // return true;
    }
    const PCLPointCloud &FrontEnd::readCurrentPointCloud() { return *filter_point_cloud_ptr; }
    const PCLPointCloud &FrontEnd::readCurrentLocalMap() { return *map_ptr->getLocalMap(); }
    bool FrontEnd::syncMeasureGroup(MeasureGroup &mg) {
        mg.imus.clear();
        mg.cloud.cloud_ptr->clear();
        if (pointcloud_deque.empty() || imu_deque.empty()) {
            return false;
        }
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double cloud_start_time = pointcloud_deque.front().time_stamp.sec();
        double cloud_end_time = pointcloud_deque.front().cloud_ptr->points.back().offset_time / 1e9 + cloud_start_time;

        if (imu_end_time < cloud_end_time) {
            // std::cout << "1" << std::endl;
            return false;
        }
        if (cloud_end_time < imu_start_time) {  //无效点云，丢掉
            pointcloud_deque.pop_front();
            // std::cout << "2" << std::endl;
            return false;
        }

        //时间戳合理
        mg.cloud = pointcloud_deque.front();
        pointcloud_deque.pop_front();
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time = cloud_end_time;
        while (!imu_deque.empty()) {
            if (imu_deque.front().time_stamp.sec() < mg.lidar_end_time) {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();
            } else {
                break;
            }
        }
        if (mg.imus.size() <= 5) {
            // std::cout << "3" << std::endl;
            return false;
        }
        return true;
    }
    void FrontEnd::initState(MeasureGroup &mg) {
        if (!use_inv) {
            static int imu_count = 0;
            static Eigen::Vector3d mean_acc{0, 0, 0};
            auto &ieskf = *ieskf_ptr;
            if (imu_inited) {
                return;
            }

            for (size_t i = 0; i < mg.imus.size(); i++) {
                imu_count++;
                auto x = ieskf.getX();
                mean_acc += mg.imus[i].acceleration;
                x.bg += mg.imus[i].gyroscope;
                ieskf.setX(x);
            }
            if (imu_count >= 5) {
                auto x = ieskf.getX();
                mean_acc /= double(imu_count);

                x.bg /= double(imu_count);
                imu_scale = GRAVITY / mean_acc.norm();
                fbpropagate_ptr->imu_scale = imu_scale;
                fbpropagate_ptr->last_imu = mg.imus.back();
                // 重力的符号为负 就和fastlio公式一致
                x.gravity = -mean_acc / mean_acc.norm() * GRAVITY;
                ieskf.setX(x);
                imu_inited = true;
            }
            return;
        } else {
            static int imu_count = 0;
            static Eigen::Vector3d mean_acc{0, 0, 0};
            static Eigen::Vector3d bg{0, 0, 0};
            auto &invkf = *invkf_ptr;  //引用
            if (imu_inited) {
                return;
            }

            for (size_t i = 0; i < mg.imus.size(); i++) {
                imu_count++;
                // auto x = invkf.getX();
                mean_acc += mg.imus[i].acceleration;
                bg += mg.imus[i].gyroscope;
                // invkf.setX(x);
            }
            if (imu_count >= 5) {
                // auto x = ieskf.getX();
                mean_acc /= double(imu_count);
                bg /= double(imu_count);
                imu_scale = GRAVITY / mean_acc.norm();
                fbpropagate_ptr->imu_scale = imu_scale;
                fbpropagate_ptr->last_imu = mg.imus.back();
                // 重力的符号为负 就和fastlio公式一致
                invkf.setGravity(-mean_acc / mean_acc.norm() * GRAVITY);
                invkf.setGyroscopeBias(bg);
                imu_inited = true;
            }
            return;
        }
    }
    IESKF::State18 FrontEnd::readState() {
        if (!use_inv) {
            return ieskf_ptr->getX();
        } else {
            IESKF::State18 x;
            x.rotation = Eigen::Quaterniond(invkf_ptr->getRotation());
            x.position = invkf_ptr->getPosition();
            //std::cout << invkf_ptr->getPosition() << '\n'<< std::endl;
            return x;
        }
    }

    IESKF::State18 FrontEnd::readState_inv() {}
}  // namespace IESKFSlam