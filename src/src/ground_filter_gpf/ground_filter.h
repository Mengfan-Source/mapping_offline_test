#ifndef GROUND_FILTER_H
#define GROUND_FILTER_H
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include <common/point_types.h>
#include <common/eigen_types.h>
#include <glog/logging.h>
namespace plane_ground_filter {
  struct PointXYZIRL {
    PCL_ADD_POINT4D;  // quad-word XYZ
    float intensity;  // laser intensity reading
    //        uint16_t ring;                  // laser ring number
    uint16_t label;                  // point label
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
  } EIGEN_ALIGN16;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    plane_ground_filter::PointXYZIRL,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(std::uint16_t, label, label))
#define SLRPointXYZIRL plane_ground_filter::PointXYZIRL
using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace xmf{
  using Eigen::JacobiSVD;
  using Eigen::MatrixXf;
  using Eigen::VectorXf;
  class PlaneGroudFilter{
    public:
    PlaneGroudFilter();
    ~PlaneGroudFilter();
    public:
    /**
     * @brief
     * @note
     * @param  sensor_model:激光的线束，如为视觉点云可设定16
     * @param  sensor_height: 点云坐标系原点距离地面高度
     * @param  clip_height: 点云上边界，裁剪高度，去除该高度以上的点云
     * @param  min_distance: 去除离传感器过近的点云
     * @param  max_distance: 点云前边界
     * @param  num_iter: 迭代次数
     * @param  num_lpr: 初始地面点个数
     * @param  num_seq: 点云划分区块个数
     * @param  th_seeds: 种子点阈值
     * @param  th_dist: 检测障碍物阈值
     * @param  height: 该高度以下点云为地面，该高度以上点云不参与地面运算
     * @param  back_distance: 点云后边界
     * @param  right_distance:点云右边界
     * @param  left_distance:点云左边界
     * @retval None
     */
    void init_params(int sensor_model, double sensor_height, double clip_height,
                    double min_distance, double max_distance, int num_iter,
                    int num_lpr, int num_seq, double th_seeds, double th_dist,
                    double height, double back_distance = -25,
                    double right_distance = 25, double left_distance = -25);
      // 处理地面函数
    void pointcloud_gpf(const CloudPtr in_cloud,CloudPtr out_no_ground);
    private:
    int sensor_model_;
    double sensor_height_, clip_height_, min_distance_, max_distance_,
        back_distance_, right_distance_,
        left_distance_;  //传感器高度、要裁剪的高度范围、距离远近
    int num_seq_ = 3;
    int num_iter_, num_lpr_;
    double th_seeds_, th_dist_, height_;

    float d_, th_dist_d_;
    MatrixXf normal_;
    CloudPtr g_seeds_pc;

    // 平面模型估计
    void estimate_plane_(void);
    void estimate_plane_(CloudPtr ground_pc);
      //    提取种子点
    void extract_initial_seeds_(const PointCloudType &p_sorted);

    // 移除位置过高的点
    void clip_above(const CloudPtr in,
                    const CloudPtr out);

    //    移除近距离，远距离噪点
    void remove_close_far_pt(const CloudPtr in,
                            const CloudPtr out);

    //    点云处理过程合计
    void post_process(const CloudPtr in,
                      CloudPtr out);

    // 分割地面
    void groundPlaneFitting(PointCloudType laserCloudIn,
                            PointCloudType laserCloudIn_org,
                            CloudPtr seg_ground,
                            CloudPtr seg_no_ground);

    void pcSegmentY(PointCloudType in_cloud,
                    std::vector<PointCloudType> &pc_segs);

    void pcSegmentX(PointCloudType in_cloud,
                    std::vector<PointCloudType> &pc_segs);

  };
}
#endif