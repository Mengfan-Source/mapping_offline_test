#ifndef XMF_IO_UTILS_H
#define XMF_IO_UTILS_H
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <fstream>
#include <functional>
#include <utility>

#include "common/dataset_type.h"
#include "common/global_flags.h"
#include "sensors/gnss.h"
#include "sensors/imu.h"
#include "sensors/lidar_utils.h"
#include "common/math_utils.h"
#include "common/message_def.h"
#include "sensors/utm_convert.h"
#include "livox_ros_driver2/CustomMsg.h"
namespace xmf
{
        class RosbagIO {
        public:
        explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
                : bag_file_(std::move(bag_file)), dataset_type_(dataset_type) {
                assert(dataset_type_ != DatasetType::UNKNOWN);
        }

        using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance &m)>;

        /// 一些方便直接使用的topics, messages
        using PointCloud2Handle = std::function<bool(sensor_msgs::PointCloud2::Ptr)>;
        using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
        using ImuHandle = std::function<bool(IMUPtr)>;
        using GNSSHandle = std::function<bool(GNSSPtr)>;
        using LivoxHandle = std::function<bool(const livox_ros_driver2::CustomMsg::ConstPtr &msg)>;

        // 遍历文件内容，调用回调函数
        void Go();

        /// 通用处理函数
        RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
                process_func_.emplace(topic_name, func);
                return *this;
        }
        /// 根据数据集类型自动确定topic名称
        RosbagIO &AddAutoPointCloudHandle(PointCloud2Handle f) {
                if (dataset_type_ == DatasetType::LIVOXMID360) {
                // MID360 不能直接获取point cloud 2
                return *this;
                } else {
                return AddHandle(GetLidarTopicName(), [f](const rosbag::MessageInstance &m) -> bool {
                        auto msg = m.instantiate<sensor_msgs::PointCloud2>();
                        if (msg == nullptr) {
                        return false;
                        }
                        return f(msg);
                });
                }
        }

        /// 根据数据集自动处理RTK消息
        RosbagIO &AddAutoRTKHandle(GNSSHandle f) {
                if (dataset_type_ == DatasetType::NCLT) {
                return AddHandle(nclt_rtk_topic, [f, this](const rosbag::MessageInstance &m) -> bool {
                        auto msg = m.instantiate<sensor_msgs::NavSatFix>();
                        if (msg == nullptr) {
                        return false;
                        }

                        GNSSPtr gnss(new GNSS(msg));
                        ConvertGps2UTMOnlyTrans(*gnss);
                        if (std::isnan(gnss->lat_lon_alt_[2])) {
                        // 貌似有Nan
                        return false;
                        }

                        return f(gnss);
                });
                } else {
                // TODO 其他数据集的RTK转换关系
                }
        }

        /// point cloud 2 的处理
        RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
                return AddHandle(topic_name, [f](const rosbag::MessageInstance &m) -> bool {
                auto msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (msg == nullptr) {
                        return false;
                }
                return f(msg);
                });
        }

        /// livox msg 处理
        RosbagIO &AddLivoxHandle(LivoxHandle f) {
                return AddHandle(GetLidarTopicName(), [f, this](const rosbag::MessageInstance &m) -> bool {
                auto msg = m.instantiate<livox_ros_driver2::CustomMsg>();
                if (msg == nullptr) {
                        LOG(INFO) << "cannot inst: " << m.getTopic();
                        return false;
                }
                return f(msg);
                });
        }
        /// IMU
        RosbagIO &AddImuHandle(ImuHandle f);

        /// 清除现有的处理函数
        void CleanProcessFunc() { process_func_.clear(); }

        private:
        /// 根据设定的数据集名称获取雷达名
        std::string GetLidarTopicName() const;

        /// 根据数据集名称确定IMU topic名称
        std::string GetIMUTopicName() const;

        std::map<std::string, MessageProcessFunction> process_func_;
        std::string bag_file_;
        public:
        DatasetType dataset_type_;
        };
} // namespace xmf

#endif