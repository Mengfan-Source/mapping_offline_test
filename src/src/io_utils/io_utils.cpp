#include"io_utils/io_utils.h"
#include <glog/logging.h>
namespace xmf
{
    std::string RosbagIO::GetLidarTopicName() const {
    if (dataset_type_ == DatasetType::NCLT) {
        return nclt_lidar_topic;
    }
    if (dataset_type_ == DatasetType::LIVOXMID360) {
        return mid360_lidar_topic;
    }
    if (dataset_type_ == DatasetType::AIRY) {
        return airy_lidar_topic;
    }
    LOG(ERROR) << "cannot load lidar topic name of dataset " << int(dataset_type_);
    return "";
}
std::string RosbagIO::GetIMUTopicName() const {
    if (dataset_type_ == DatasetType::LIVOXMID360) {
        return mid360_imu_topic;
    } else if (dataset_type_ == DatasetType::NCLT) {
        return nclt_imu_topic;
    } 
    else if (dataset_type_ == DatasetType::AIRY) {
        return airy_imu_topic;
    } 
    else {
        LOG(ERROR) << "cannot load imu topic name of dataset " << int(dataset_type_);
    }

return "";
}

void RosbagIO::Go() {
    rosbag::Bag bag(bag_file_);
    LOG(INFO) << "running in " << bag_file_ << ", reg process func: " << process_func_.size();

    if (!bag.isOpen()) {
        LOG(ERROR) << "cannot open " << bag_file_;
        return;
    }

    auto view = rosbag::View(bag);
    for (const rosbag::MessageInstance &m : view) {
        auto iter = process_func_.find(m.getTopic());
        if (iter != process_func_.end()) {
            iter->second(m);
        }

        if (global::FLAG_EXIT) {
            break;
        }
    }

    bag.close();
    LOG(INFO) << "bag " << bag_file_ << " finished.";
}

RosbagIO &RosbagIO::AddImuHandle(RosbagIO::ImuHandle f) {
    return AddHandle(GetIMUTopicName(), [&f, this](const rosbag::MessageInstance &m) -> bool {
        auto msg = m.template instantiate<sensor_msgs::Imu>();
        if (msg == nullptr) {
            return false;
        }

        IMUPtr imu;
        if (dataset_type_ == DatasetType::LIVOXMID360) {
            // Livox内置imu的加计需要乘上重力常数
            imu =
                std::make_shared<IMU>(msg->header.stamp.toSec(),
                                      Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                    //   Vec3d(msg->linear_acceleration.x * 9.80665, msg->linear_acceleration.y * 9.80665,
                                    //         msg->linear_acceleration.z * 9.80665));
                                    Vec3d(msg->linear_acceleration.x , msg->linear_acceleration.y,
                                            msg->linear_acceleration.z ));
        } 
        else if (dataset_type_ == DatasetType::AIRY){
            //
            // imu =std::make_shared<IMU>(msg->header.stamp.toSec(),
            //                           Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
            //                           Vec3d(msg->linear_acceleration.x * 9.80665, msg->linear_acceleration.y * 9.80665,
            //                                 msg->linear_acceleration.z * 9.80665));
            //处理坐标变换和量纲
            Quatd q_lidar2imu(0.0029166745953261852,0.7073081731796265,-0.7068824768066406,0.004880243446677923);
            Vec3d t_lidar2imu(0.0042, 0.0041, -0.0044);
            Eigen::Isometry3d T_imu2lidar = Eigen::Isometry3d::Identity();
            T_imu2lidar.rotate(q_lidar2imu.toRotationMatrix().inverse());
            T_imu2lidar.pretranslate(-t_lidar2imu);
            // Vec3d acc_imu(msg->linear_acceleration.x * 9.80665,msg->linear_acceleration.y * 9.80665,msg->linear_acceleration.z * 9.80665);
            Vec3d acc_imu(msg->linear_acceleration.x * 9.81,msg->linear_acceleration.y * 9.81,msg->linear_acceleration.z * 9.81);
            Vec3d gyro_imu(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
            Vec3d acc_lidar = T_imu2lidar * acc_imu;
            Vec3d gyro_lidar = T_imu2lidar * gyro_imu;
            imu =std::make_shared<IMU>(msg->header.stamp.toSec(),gyro_lidar,acc_lidar);
            // LOG(INFO)<<"run into dataset AIRY";
        }
        else {
            imu = std::make_shared<IMU>(
                msg->header.stamp.toSec(),
                Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
        }
        return f(imu);
    });
}

} // namespace xmf

