#pragma once
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/modules/invkf/invkf.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/math/geometry.h"

namespace IESKFSlam{
    class LIOZHModel : public IESKF::CalcZHInterface{
        
        private:
        const int NEAR_POINTS_NUM = 5;
        using loss_type = triple<Eigen::Vector3d, Eigen::Vector3d, double>;
        KDTree::ConstPtr global_map_kdtree_ptr;
        PCLPointCloudPtr current_cloud_ptr;
        PCLPointCloudConstPtr local_map_ptr;
        public:
        using Ptr = std::shared_ptr<LIOZHModel>;
        void prepare(KDTree::ConstPtr kd_tree, PCLPointCloudPtr current_cloud, PCLPointCloudConstPtr local_map){
            global_map_kdtree_ptr = kd_tree;
            current_cloud_ptr = current_cloud;
            local_map_ptr = local_map;
        }
        bool calculate(const IESKF::State18 &state, Eigen::MatrixXd &Z, Eigen::MatrixXd &H) override{
            std::vector<loss_type> loss_v;
            loss_v.resize(current_cloud_ptr->size());//点云size
            std::vector<bool> is_effect_point(current_cloud_ptr->size(), false);//初始化全部点云状态为false
            std::vector<loss_type> loss_real;
            int valid_points_num = 0;
            #ifdef MP_EN
                omp_set_num_threads(MP_PROC_NUM);
                #pragma omp parallel for
            #endif
                        /**
             * 有效点的判断
             * 1. 将当前点变换到世界系下
             * 2. 在局部地图上使用kd_tree 临近搜索NEAR_POINTS_NUM个点
             * 3. 判断这些点是否构成平面
             * 4. 判断点离这个平面够不够近(达到阈值)
             * 5. 满足上述条件，设置为有效点。
            */
            for (size_t  i = 0; i < current_cloud_ptr->size(); i++) //这个耗时会很高把？
            {
                // . 变换到世界系
                Point point_imu = current_cloud_ptr->points[i];
                Point point_world;
                point_world = transformPoint(point_imu,state.rotation,state.position);
                // . 临近搜索
                std::vector<int> point_ind;
                std::vector<float> distance;
                global_map_kdtree_ptr->nearestKSearch(point_world,NEAR_POINTS_NUM,point_ind,distance);
                // . 是否搜索到足够的点以及最远的点到当前点的距离足够小(太远，就不认为这俩在一个平面)
                if (distance.size()<NEAR_POINTS_NUM||distance[NEAR_POINTS_NUM-1]>5)
                {
                    continue;
                }
                // . 判断这些点够不够成平面
                std::vector<Point> planar_points;
                for (int ni = 0; ni < NEAR_POINTS_NUM; ni++)
                {
                    planar_points.push_back(local_map_ptr->at(point_ind[ni]));
                }
                Eigen::Vector4d pabcd;
                // . 如果构成平面
                if (planarCheck(planar_points,pabcd,0.1))//判断平面，如果是平面，true，其系数abcd也会得到    
                {
                    // . 计算点到平面距离
                    double pd = point_world.x*pabcd(0)+point_world.y*pabcd(1)+point_world.z*pabcd(2)+pabcd(3);
                    // . 记录残差
                    loss_type loss;
                    loss.thrid = pd; // 残差
                    loss.first = {point_imu.x,point_imu.y,point_imu.z}; // imu系下点的坐标，用于求H
                    loss.second = pabcd.block<3,1>(0,0);// 平面法向量 用于求H
                    if (isnan(pd)||isnan(loss.second(0))||isnan(loss.second(1))||isnan(loss.second(2)))continue;
                    // .计算点和平面的夹角，夹角越小S越大。
                    double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm());
                    if(s > 0.9 ){
                        valid_points_num++;
                        loss_v[i]=(loss);
                        is_effect_point[i] = true;
                    }
                }

            }
            for (size_t i = 0; i <current_cloud_ptr->size() ; i++)
            {
                if(is_effect_point[i])loss_real.push_back(loss_v[i]);
            }
            // 根据有效点的数量分配H Z的大小
            valid_points_num = loss_real.size();//真正有效的点;
            H = Eigen::MatrixXd::Zero(valid_points_num, 18); 
            Z.resize(valid_points_num,1);
            for (int vi = 0; vi < valid_points_num; vi++)
            {
                // H 记录导数
                Eigen::Vector3d dr = -1*loss_real[vi].second.transpose()*state.rotation.toRotationMatrix()*skewSymmetric(loss_real[vi].first);
                H.block<1,3>(vi,0) = dr.transpose();
                H.block<1,3>(vi,3) = loss_real[vi].second.transpose();
                // Z记录距离
                Z(vi,0) = loss_real[vi].thrid;
            }
            return true;
        }
    };
    class LIOZHModelINV :public INVKF::CalcZHInterface{
        private:
        const int NEAR_POINTS_NUM = 5;
        using loss_type = triple<Eigen::Vector3d, Eigen::Vector3d, double>;
        KDTree::ConstPtr global_map_kdtree_ptr;
        PCLPointCloudPtr current_cloud_ptr;
        PCLPointCloudConstPtr local_map_ptr;
        public:
        using Ptr = std::shared_ptr<LIOZHModelINV>;
        void prepare(KDTree::ConstPtr kd_tree, PCLPointCloudPtr current_cloud, PCLPointCloudConstPtr local_map){
            global_map_kdtree_ptr = kd_tree;
            current_cloud_ptr = current_cloud;
            local_map_ptr = local_map;
        }
        bool calculate(Eigen::Matrix<double,5,5> &state, Eigen::MatrixXd &Z, Eigen::MatrixXd &H) override{
            Eigen::Matrix3d rotation_mat = state.block(0,0,3,3);
            Eigen::Quaterniond rotation = Eigen::Quaterniond(rotation_mat);
            //rotation.normalize();
            Eigen::Vector3d position = state.block(0,4,3,1);
            // Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
            // Eigen::Vector3d position =  Eigen::Vector3d(0,0,0);
            std::vector<loss_type> loss_v;
            loss_v.resize(current_cloud_ptr->size());//点云size
            std::vector<bool> is_effect_point(current_cloud_ptr->size(), false);//初始化全部点云状态为false
            std::vector<loss_type> loss_real;
            int valid_points_num = 0;
            #ifdef MP_EN
                omp_set_num_threads(MP_PROC_NUM);
                #pragma omp parallel for
            #endif
                        /**
             * 有效点的判断
             * 1. 将当前点变换到世界系下
             * 2. 在局部地图上使用kd_tree 临近搜索NEAR_POINTS_NUM个点
             * 3. 判断这些点是否构成平面
             * 4. 判断点离这个平面够不够近(达到阈值)
             * 5. 满足上述条件，设置为有效点。
            */
            for (size_t  i = 0; i < current_cloud_ptr->size(); i++) //这个耗时会很高把？
            {
                // . 变换到世界系
                Point point_imu = current_cloud_ptr->points[i];
                Point point_world;
                //point_world = -1 * r_position - rotation * point_imu.;
                //Eigen::Matrix<double,3,1> ep = {point_imu.x,point_imu.y,point_imu.z};
                //point_world = -1 * r_position - rotation * ep;
                point_world = transformPoint(point_imu,rotation,position);
                // . 临近搜索
                std::vector<int> point_ind;
                std::vector<float> distance;
                if (!pcl_isfinite(point_world.x) || !pcl_isfinite(point_world.y) || !pcl_isfinite(point_world.z) || !pcl_isfinite(point_world.intensity))
{
  continue;
}

                
                global_map_kdtree_ptr->nearestKSearch(point_world,NEAR_POINTS_NUM,point_ind,distance);
                // . 是否搜索到足够的点以及最远的点到当前点的距离足够小(太远，就不认为这俩在一个平面)
                if (distance.size()<NEAR_POINTS_NUM||distance[NEAR_POINTS_NUM-1]>5)
                {
                    continue;
                }
                // . 判断这些点够不够成平面
                std::vector<Point> planar_points;
                for (int ni = 0; ni < NEAR_POINTS_NUM; ni++)
                {
                    planar_points.push_back(local_map_ptr->at(point_ind[ni]));
                }
                Eigen::Vector4d pabcd;
                // . 如果构成平面
                if (planarCheck(planar_points,pabcd,0.1))//判断平面，如果是平面，true，其系数abcd也会得到    
                {
                    // . 计算点到平面距离
                    double pd = point_world.x*pabcd(0)+point_world.y*pabcd(1)+point_world.z*pabcd(2)+pabcd(3);
                    // . 记录残差
                    loss_type loss;
                    
                    loss.thrid = -1 * pd; // 残差
                    loss.first = {point_world.x,point_world.y,point_world.z}; // imu系下点的坐标，用于求H
                    loss.second = pabcd.block<3,1>(0,0);// 平面法向量 用于求H
                    if (isnan(pd)||isnan(loss.second(0))||isnan(loss.second(1))||isnan(loss.second(2)))continue;
                    // .计算点和平面的夹角，夹角越小S越大。
                    double s = 1 - 0.9 * fabs(pd) / sqrt(loss.first.norm());
                    if(s > 0.9 ){
                        valid_points_num++;
                        loss_v[i]=(loss);
                        is_effect_point[i] = true;
                    }
                }

            }
            for (size_t i = 0; i <current_cloud_ptr->size() ; i++)
            {
                if(is_effect_point[i])loss_real.push_back(loss_v[i]);
            }
            // 根据有效点的数量分配H Z的大小
            valid_points_num = loss_real.size();//真正有效的点;
            H = Eigen::MatrixXd::Zero(valid_points_num, 18); 
            Z.resize(valid_points_num,1);
            for (int vi = 0; vi < valid_points_num; vi++)
            {
                // H 记录导数
                Eigen::Vector3d dr = -1*loss_real[vi].second.transpose()*skewSymmetric(loss_real[vi].first);
                H.block<1,3>(vi,0) = dr.transpose();
                H.block<1,3>(vi,6) = loss_real[vi].second.transpose();
                // Z记录距离
                Z(vi,0) = loss_real[vi].thrid;
            }
            return true;
        }
    };
}
