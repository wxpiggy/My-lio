#pragma once
#include <eq_lio/type/pointcloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "/root/ws_livox/src/I-RIKF-LIO/lib/ikd_Tree/ikd_Tree.h"

//#include "eq_lio/type/point.h"
namespace EQLIO {
    // 体素滤波器
    using VoxelFilter = pcl::VoxelGrid<Point>;
    // KDTree
    using KDTree   = KD_TREE<Point>;
    using KDTreePtr =  KDTree::Ptr;
    // using IKDTreeConstPtr = IKDTree::
    // using KDTree = pcl::KdTreeFLANN<Point>;
    // using KDTreePtr = KDTree::Ptr;
    // using KDTreeConstPtr = KDTree::ConstPtr;
    // 定义重力常量
    const double GRAVITY = 9.81;
    template<typename _first, typename _second, typename _thrid>
    struct triple{
        _first first;
        _second second;
        _thrid thrid;
    };
}  // namespace EQLIO
