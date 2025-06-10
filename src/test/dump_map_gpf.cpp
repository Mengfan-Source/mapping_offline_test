// #include <gflags/gflags.h>
// #include <glog/logging.h>

// DEFINE_double(voxel_size, 0.1, "导出地图分辨率");
// DEFINE_string(pose_source, "lidar", "使用的pose来源:lidar/rtk/opti1/opti2");
// DEFINE_string(dump_to, "./../data/mapdata/", "导出的目标路径");

// #include <pcl/common/transforms.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/io/pcd_io.h>

// #include "frontend/keyframe.h"
// #include "common/point_cloud_utils.h"
// #include "ground_filter_gpf/ground_filter.h"

// /**
//  * 将keyframes.txt中的地图和点云在base坐标系下去除Z值以下的点并合并为一个pcd
//  */

// int main(int argc, char** argv) {
//     FLAGS_log_dir = "./../log";
//     google::ParseCommandLineFlags(&argc, &argv, true);
//     google::InitGoogleLogging(argv[0]);
//     FLAGS_stderrthreshold = google::INFO;
//     FLAGS_colorlogtostderr = true;
//     int sensor_model;
//     double sensor_height, clip_height, min_distance, front_distance,back_distance, right_distance,left_distance;  //传感器高度、要裁剪的高度范围、距离远近
//     int num_iter, num_lpr, num_seq;
//     double th_seeds, th_dist, height;

//     sensor_model = 16;//<!-- we use 16 lidar -->
//     sensor_height = 0.8;//<!-- the height of lidar position  -->
//     clip_height = 4.0;//<!-- clip the points above sensor_height+clip_height -->
//     min_distance = 0.5;//<!-- the min distance to be remove -->
//     front_distance = 20.0;//<!-- the front distance to be remove -->
//     num_iter = 5;//<!-- Num of Iteration SVD的次数-->
//     num_lpr = 400;// <!-- Num of LPR  选取LPR最低高度点的数量-->
//     num_seq = 1;//<!-- Num of Seq 地面划分的区域数-->
//     th_seeds = 0.3;//<!-- Seeds Threshold 选取种子点的阀值-->
//     th_dist = 0.1;//<!-- Distance Threshold 平面距离阀值-->
//     height = 0.2;//<!-- Height Threshold 地面高度阀值-->
//     back_distance = -20;//<!-- the back distance to be remove -->
//     right_distance = 20;//<!-- the right distance to be remove -->
//     left_distance = -20;//<!-- the left distance to be remove -->

//     xmf::PlaneGroudFilter planeGroudFilter;
//     planeGroudFilter.init_params(sensor_model, sensor_height, clip_height,
//                                min_distance, front_distance, num_iter, num_lpr,
//                                num_seq, th_seeds, th_dist, height,
//                                back_distance, right_distance, left_distance);
//     using namespace xmf;
//     std::map<IdType, KFPtr> keyframes;
//     if (!LoadKeyFrames("./../data/mapdata/keyframes.txt", keyframes)) {
//         LOG(ERROR) << "failed to load keyframes.txt";
//         return -1;
//     }

//     if (keyframes.empty()) {
//         LOG(INFO) << "keyframes are empty";
//         return 0;
//     }

//     // dump kf cloud and merge
//     LOG(INFO) << "merging";
//     CloudPtr global_cloud(new PointCloudType);

//     pcl::VoxelGrid<PointType> voxel_grid_filter;
//     float resolution = FLAGS_voxel_size;
//     voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

//     int cnt = 0;
//     for (auto& kfp : keyframes) {
//         auto kf = kfp.second;
//         SE3 pose;
//         if (FLAGS_pose_source == "rtk") {
//             pose = kf->rtk_pose_;
//         } else if (FLAGS_pose_source == "lidar") {
//             pose = kf->lidar_pose_;
//         } else if (FLAGS_pose_source == "opti1") {
//             pose = kf->opti_pose_1_;
//         } else if (FLAGS_pose_source == "opti2") {
//             pose = kf->opti_pose_2_;
//         }

//         kf->LoadScan("./../data/mapdata/");

//         CloudPtr cloud_trans(new PointCloudType);
//         RemoveGround(kf->cloud_,-0.3);
//         pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix());

//         // voxel size
//         CloudPtr kf_cloud_voxeled(new PointCloudType);
//         voxel_grid_filter.setInputCloud(cloud_trans);
//         voxel_grid_filter.filter(*kf_cloud_voxeled);

//         *global_cloud += *kf_cloud_voxeled;
//         kf->cloud_ = nullptr;

//         LOG(INFO) << "merging " << cnt << " in " << keyframes.size() << ", pts: " << kf_cloud_voxeled->size()
//                   << " global pts: " << global_cloud->size();
//         cnt++;
//     }

//     if (!global_cloud->empty()) {
//         // xmf::VoxelGrid(global_cloud,0.1);
//         xmf::SaveCloudToFile(FLAGS_dump_to + "/map_noground.pcd", *global_cloud);
//     }

//     LOG(INFO) << "done.";
//     return 0;
// }


#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_double(voxel_size, 0.1, "导出地图分辨率");
DEFINE_string(pose_source, "lidar", "使用的pose来源:lidar/rtk/opti1/opti2");
DEFINE_string(dump_to, "./../data/mapdata/", "导出的目标路径");

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "frontend/keyframe.h"
#include "common/point_cloud_utils.h"
#include "ground_filter_gpf/ground_filter.h"

/**
 * 将keyframes.txt中的地图和点云在base坐标系下去除Z值以下的点并合并为一个pcd
 */

int main(int argc, char** argv) {
    FLAGS_log_dir = "./../log";
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    int sensor_model;
    double sensor_height, clip_height, min_distance, front_distance,back_distance, right_distance,left_distance;  //传感器高度、要裁剪的高度范围、距离远近
    int num_iter, num_lpr, num_seq;
    double th_seeds, th_dist, height;

    sensor_model = 16;//<!-- we use 16 lidar -->
    sensor_height = 0.5;//<!-- the height of lidar position  -->
    clip_height = 4.0;//<!-- clip the points above sensor_height+clip_height -->
    min_distance = 0.5;//<!-- the min distance to be remove -->
    front_distance = 20.0;//<!-- the front distance to be remove -->
    num_iter = 5;//<!-- Num of Iteration SVD的次数-->
    num_lpr = 40000;// <!-- Num of LPR  选取LPR最低高度点的数量-->//400
    num_seq = 1;//<!-- Num of Seq 地面划分的区域数-->
    th_seeds = 0.3;//<!-- Seeds Threshold 选取种子点的阀值-->
    th_dist = 0.2;//<!-- Distance Threshold 平面距离阀值-->
    height = 0.2;//<!-- Height Threshold 地面高度阀值-->
    back_distance = -20;//<!-- the back distance to be remove -->
    right_distance = 20;//<!-- the right distance to be remove -->
    left_distance = -20;//<!-- the left distance to be remove -->
    using namespace xmf;
    PlaneGroudFilter planeGroudFilter;
    planeGroudFilter.init_params(sensor_model, sensor_height, clip_height,
                               min_distance, front_distance, num_iter, num_lpr,
                               num_seq, th_seeds, th_dist, height,
                               back_distance, right_distance, left_distance);
    
    CloudPtr cloud_(new PointCloudType);
    CloudPtr cloud_processed_(new PointCloudType);
    pcl::io::loadPCDFile("/home/xmf/xmf_slam/mapping_offline_test/data/tempout/800.pcd", *cloud_);
    planeGroudFilter.pointcloud_gpf(cloud_,cloud_processed_);
    SaveCloudToFile("/home/xmf/xmf_slam/mapping_offline_test/data/tempout/800_p.pcd", *cloud_processed_);
    LOG(INFO) << "done.";
    return 0;
}

