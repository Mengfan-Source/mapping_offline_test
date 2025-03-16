#ifndef XMF_IMU_H
#define XMF_IMU_H

#include <memory>
#include "common/eigen_types.h"

namespace xmf {

/// IMU 读数
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d& acce) : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Vec3d gyro_ = Vec3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};
}  // namespace xmf

using IMUPtr = std::shared_ptr<xmf::IMU>;

#endif  // XMF_IMU_H
