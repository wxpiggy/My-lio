#pragma once
#include <deque>

#include "eq_lio/type/imu.h"
#include "eq_lio/type/pointcloud.h"
namespace EQLIO {
    struct MeasureGroup {
        double lidar_begin_time;
        double lidar_end_time;
        std::deque<IMU> imus;
        PointCloud cloud;
    };

}  // namespace EQLIO