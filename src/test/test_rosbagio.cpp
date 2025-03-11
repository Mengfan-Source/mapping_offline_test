#include <iostream>
#include <string>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "io_utils/io_utils.h"
#include "sensors/cloud_convert.h"

int main(int argc,char ** argv){
        int i = 0;
        FLAGS_log_dir = "./../log";
        google::InitGoogleLogging(argv[0]);
        FLAGS_stderrthreshold = google::INFO;//设置日志输出级别
        FLAGS_colorlogtostderr = true;
        google::ParseCommandLineFlags(&argc, &argv, true);//解析命令行参数
        LOG(INFO) << "mapping offline frontend";
        // xmf::RosbagIO rosbag_io("/home/cetc21/Downloads/slam-learning/NCLT/20130110.bag",xmf::DatasetType::NCLT);
        xmf::RosbagIO rosbag_io("/home/cetc21/xmf/tempout/bags/dahei_mid360/dahei_mid360_zou_custom.bag",xmf::DatasetType::LIVOXMID360);
        xmf::CloudConvert cloudconvert;//mid360激光雷达转为全量点云
        // cloudconvert.Mid360Handler()
        rosbag_io.AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool{
                xmf::PointCloudType cloudxyzi;
                pcl::fromROSMsg(*cloud, cloudxyzi);
                i++;
                pcl::io::savePCDFileASCII("/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/tempout/"+std::to_string(i)+".pcd", cloudxyzi);
                return true;
        }).AddLivoxHandle([&](const livox_ros_driver2::CustomMsg::ConstPtr &msg) -> bool{
                xmf::FullCloudPtr ptr(new xmf::FullPointCloudType());
                cloudconvert.Process(msg,ptr);
                xmf::CloudPtr ptrxyzi(new xmf::PointCloudType);
                ptrxyzi = xmf::ConvertToCloud<xmf::FullPointType>(ptr);
                i++;
                ptrxyzi->height = 1;
                ptrxyzi->width = ptrxyzi->size();
                std::cout<<"Lidar Time:"<<msg->header.stamp.toSec()<<std::endl;
                pcl::io::savePCDFileASCII("/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/tempout/"+std::to_string(i)+".pcd", *ptrxyzi);
                return true;
        }).AddImuHandle([&](IMUPtr imu) ->bool{
                std::cout<<"IMU Time"<<imu->timestamp_<<std::endl;
                std::cout<<imu->acce_<<std::endl;
                std::cout<<imu->gyro_<<std::endl;
                return true;
        }).Go();
        //imu->timestamp_单位是秒
        return 0;
}
//开发过程中的零星功能测试代码
/*#include <iostream>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include "common/eigen_types.h"
#include "common/dataset_type.h"
#include "sensors/imu.h"
#include "sensors/gnss.h"
#include "sensors/lidar_utils.h"
#include "common/math_utils.h"
#include "common/global_flags.h"
#include "pointcloud_convert/velodyne_convertor.h"
#include "io_utils/io_utils.h"
void printDatasetType(xmf::DatasetType type) {
    switch (type) {
        case xmf::DatasetType::UNKNOWN:
            std::cout << "DatasetType::UNKNOWN" << std::endl;
            break;
        case xmf::DatasetType::NCLT:
            std::cout << "DatasetType::NCLT - NCLT Dataset" << std::endl;
            break;
        case xmf::DatasetType::KITTI:
            std::cout << "DatasetType::KITTI - KITTI Dataset" << std::endl;
            break;
        case xmf::DatasetType::ULHK:
            std::cout << "DatasetType::ULHK - UrbanLoco Dataset" << std::endl;
            break;
        case xmf::DatasetType::UTBM:
            std::cout << "DatasetType::UTBM - UTBM Dataset" << std::endl;
            break;
        case xmf::DatasetType::AVIA:
            std::cout << "DatasetType::AVIA - Avia Dataset" << std::endl;
            break;
        case xmf::DatasetType::WXB_3D:
            std::cout << "DatasetType::WXB_3D - 3D Wxb Dataset" << std::endl;
            break;
        default:
            std::cout << "Unknown DatasetType" << std::endl;
    }
}
int main(){
        Mat4d a;
        a<<-0.00355503,0.0196655,0.9998,0.105433,
                                        0.999876,-0.0152523,0.00385531,0.0388548,
                                        0.0153251,0.99969,-0.0196088,-0.0375546,
                                        0,0,0,1;
        std::cout<<a<<std::endl;

        for (int i = -1; i <= 6; ++i) { // 假设 WXB_3D 的值是 6
                printDatasetType(static_cast<xmf::DatasetType>(i));
        }
        xmf::IMU imu;
        imu.gyro_ = Vec3d(2,5,8);
        std::cout<<imu.gyro_<<std::endl;
        xmf::GNSS gnss;
        gnss.heading_ = 5;
        std::cout<<gnss.heading_<<std::endl;

        xmf::CloudPtr cloud(new xmf::PointCloudType);
        pcl::io::loadPCDFile( "/home/cetc21/xmf/my_slam_ws/slam_test/data/2mid3601024/scans110.pcd",*cloud);
        pcl::visualization::PCLVisualizer::Ptr viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
                        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>color_z1 (cloud, "z");
                        viewer1->setBackgroundColor (0, 0, 0);
                        viewer1->addPointCloud<pcl::PointXYZI> (cloud, color_z1,"target_points");
                        viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,"target_points");
                        viewer1->initCameraParameters ();
                        while (!viewer1->wasStopped ()) {
                                viewer1->spinOnce ();
                        }
        std::cout<<xmf::math::G_m_s2<<std::endl;
        return 0;
}*/