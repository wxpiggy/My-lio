#pragma once
#include "ieskf_slam/type/base_type.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
namespace ROSNoetic {
    class CommonLidarProcessInterface{
        public:
            virtual bool process(const sensor_msgs::PointCloud2 &msg, IESKFSlam::PointCloud &cloud) = 0;
    };
}