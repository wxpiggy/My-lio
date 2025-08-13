#pragma once
#include "eq_lio/type/point.h"
#include "eq_lio/type/timestamp.h"

namespace EQLIO {
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = PCLPointCloud::Ptr;
    using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;
    struct PointCloud {
        using Ptr = std::shared_ptr<PointCloud>;  //指针别名
        TimeStamp time_stamp;                     //时间戳
        PCLPointCloudPtr cloud_ptr;               // pcl
        PointCloud() { cloud_ptr = pcl::make_shared<PCLPointCloud>(); }
    };
    
}  // namespace EQLIO