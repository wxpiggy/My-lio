#pragma once
#include "eq_lio/modules/module_base.h"
#include "eq_lio/type/pointcloud.h"
#include "eq_lio/type/base_type.h"
#include "pcl/common/transforms.h"
#include "eq_lio/math/math.h"

namespace EQLIO {
    class RectMapManager : private ModuleBase {
    private:
        PCLPointCloudPtr local_map_ptr;
        PCLPointCloudPtr global_map_ptr;
        KDTreePtr kdtree_ptr;
        float map_side_length_2;
        float map_resolution;
    public:
        RectMapManager(const std::string &config_path,
                       const std::string &prefix);
        ~RectMapManager();
        void reset();
        void addScan(PCLPointCloudPtr curr_scan,
                     const Eigen::Quaterniond &att_q,
                     const Eigen::Vector3d &pos_t);
        PCLPointCloudConstPtr getLocalMap();
        PCLPointCloudConstPtr getGlobalMap();
        const KDTreePtr readKDtree() ;
        // KDTreeConstPtr readKDtree();
    };
}  // namespace EQLIO