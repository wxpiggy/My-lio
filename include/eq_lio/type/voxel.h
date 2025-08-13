#pragma once
#include <cstdint>
#include <vector>
#include <Eigen/Dense>
namespace EQLIO {
    class VOXEL_LOC {
    public:
        int64_t x, y, z;

        VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

        bool operator==(const VOXEL_LOC &other) const { return (x == other.x && y == other.y && z == other.z); }
    };
}  // namespace EQLIO
