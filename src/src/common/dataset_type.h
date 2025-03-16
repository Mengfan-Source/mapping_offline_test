
#ifndef XMF_DATASET_TYPE_H
#define XMF_DATASET_TYPE_H
namespace xmf {
enum class DatasetType {
    UNKNOWN = -1,
    NCLT = 0,   // NCLT: http://robots.engin.umich.edu/nclt/
    LIVOXMID360 = 1,  // mid360激光雷达
};

inline DatasetType Str2DatasetType(const std::string& name) {
    if (name == "NCLT") {
        return DatasetType::NCLT;
    }
    if (name == "LIVOXMID360") {
        return DatasetType::LIVOXMID360;
    }
    return DatasetType::UNKNOWN;
}
/// 各种数据集里用的topic名称
const std::string nclt_rtk_topic = "gps_rtk_fix";
const std::string nclt_imu_topic = "imu_raw";
const std::string nclt_lidar_topic = "points_raw";

const std::string mid360_lidar_topic = "/livox/lidar";
const std::string mid360_imu_topic = "/imu/data";

}  

#endif  
