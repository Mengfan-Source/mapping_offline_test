#include "ground_filter_gpf/ground_filter.h"
namespace xmf{
    struct rect_in {
        double rect_min_x;
        double rect_min_y;
        double rect_max_x;
        double rect_max_y;

            public:
        rect_in &operator=(const rect_in &in) {
            if (this != &in) {
            this->rect_max_x = in.rect_max_x;
            this->rect_max_y = in.rect_max_y;
            this->rect_min_x = in.rect_min_x;
            this->rect_min_y = in.rect_min_y;
            }
            return *this;
        }
    };
    /**
    * @FuncName      remove_outside_rect
    * @Brief         除去距离传感器中心矩形范围外的点云，保留矩形范围内的点云
    */
    template <typename T>
    void remove_outside_rect(typename pcl::PointCloud<T>::Ptr in_cloud,
                            typename pcl::PointCloud<T>::Ptr out_cloud,
                            double min_x, double min_y, double max_x,
                            double max_y) {
    typename pcl::ExtractIndices<T> cliper;

    // rect_in rect_save_ = {left_distance_, back_distance_, right_distance_,
    // max_distance_};
    rect_in rect_save_ = {min_x, min_y, max_x, max_y};
    cliper.setInputCloud(in_cloud);
    pcl::PointIndices indices;
    //    ROS_INFO_STREAM("****** rect: " << rect_save_.rect_min_x << " , " <<
    //    rect_save_.rect_min_y);
    #pragma omp for
    for (size_t i = 0; i < in_cloud->points.size(); i++) {
        double x = in_cloud->points[i].x;
        double y = in_cloud->points[i].y;
        if (rect_save_.rect_min_x > x || y < rect_save_.rect_min_y ||
            rect_save_.rect_max_x < x || y > rect_save_.rect_max_y) {
        indices.indices.push_back(i);
        }
    }
    //    ROS_INFO_STREAM("****** indices size: " << indices.indices.size());
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);  // ture to remove the indices
    cliper.filter(*out_cloud);
    }

    PlaneGroudFilter::PlaneGroudFilter() {
        g_seeds_pc =
            pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);  //种子集
        //    g_ground_pc = pcl::PointCloud<VPoint>::Ptr (new
        //    pcl::PointCloud<VPoint>);//地面点集
        //    g_not_ground_pc = pcl::PointCloud<VPoint>::Ptr (new
        //    pcl::PointCloud<VPoint>);//非地面点集
        //    g_all_pc = pcl::PointCloud<SLRPointXYZIRL>::Ptr(new
        //    pcl::PointCloud<SLRPointXYZIRL>); //所有点集
    }
    PlaneGroudFilter::~PlaneGroudFilter() {}
    void PlaneGroudFilter::init_params(int sensor_model, double sensor_height,
                                   double clip_height, double min_distance,
                                   double max_distance, int num_iter,
                                   int num_lpr, int num_seq, double th_seeds,
                                   double th_dist, double height,
                                   double back_distance, double right_distance,
                                   double left_distance) {
        sensor_model_ = sensor_model;
        sensor_height_ = sensor_height;
        clip_height_ = clip_height;
        min_distance_ = min_distance;
        max_distance_ = max_distance;
        back_distance_ = back_distance;
        right_distance_ = right_distance;
        left_distance_ = left_distance;

        num_iter_ = num_iter;
        num_lpr_ = num_lpr;
        num_seq_ = num_seq;
        th_seeds_ = th_seeds;
        th_dist_ = th_dist;
        height_ = height;
        // ROS_INFO_STREAM("sensor_height_: " << sensor_height_<< " th_seeds_:"<<th_seeds_<< " th_dist_:"<<th_dist_<<" height_:"<<height_);
    }
    void PlaneGroudFilter::pointcloud_gpf(const CloudPtr in_cloud,CloudPtr out_no_ground) {
        CloudPtr laserCloudIn(new PointCloudType);
        PointType point;
        // #pragma omp for
        for (size_t i = 0; i < in_cloud->points.size(); i++) {
            if (in_cloud->points[i].z < (-1.0 * sensor_height_ + height_)) {
                point.x = in_cloud->points[i].x;
                point.y = in_cloud->points[i].y;
                point.z = in_cloud->points[i].z;
                // point.intensity = 0.8;
                point.intensity = in_cloud->points[i].intensity;
                laserCloudIn->points.push_back(point);  //获取地面点
            } else {
                PointType pointXyz;
                pointXyz.x = in_cloud->points[i].x;
                pointXyz.y = in_cloud->points[i].y;
                pointXyz.z = in_cloud->points[i].z;
                pointXyz.intensity = in_cloud->points[i].intensity;
                // ROS_DEBUG_STREAM("****** point z: " << in_cloud->points[i].z);
                out_no_ground->points.push_back(pointXyz);//
            }
        }
        // ROS_DEBUG_STREAM("****** laserCloudIn size: " << laserCloudIn->points.size());
        CloudPtr part_no_ground(new PointCloudType());
        CloudPtr part_ground(new PointCloudType());
        PointCloudType temp;
        pcl::copyPointCloud(*laserCloudIn, temp);

        groundPlaneFitting(*laserCloudIn, temp, part_ground, part_no_ground);

        PointType pointXyz;
        for (size_t i = 0; i < part_no_ground->points.size(); i++) {
            pointXyz.x = part_no_ground->points[i].x;
            pointXyz.y = part_no_ground->points[i].y;
            pointXyz.z = part_no_ground->points[i].z;
            pointXyz.intensity = part_no_ground->points[i].intensity;
            out_no_ground->points.push_back(pointXyz);
        }
    }

    /*void PlaneGroudFilter::point_gpf(const sensor_msgs::PointCloud2ConstPtr &in_cloud,pcl::PointCloud<VPoint>::Ptr final_ground,pcl::PointCloud<VPoint>::Ptr final_no_ground) {
        // PointCloud2 转 PointCloud<VPoint>
        pcl::PointCloud<VPoint>::Ptr laserCloudIntemp(new pcl::PointCloud<VPoint>);
        pcl::PointCloud<VPoint>::Ptr laserCloudIn(new pcl::PointCloud<VPoint>);
        pcl::fromROSMsg(*in_cloud, *laserCloudIntemp);

        for (size_t i = 0; i < laserCloudIntemp->points.size(); i++) {
            if (laserCloudIntemp->points[i].z < (-1.0 * sensor_height_ + height_)) {
            laserCloudIn->points.push_back(
                laserCloudIntemp->points[i]);  //获取所有的点
            } else {
            final_no_ground->points.push_back(laserCloudIntemp->points[i]);
            }
        }

        std::vector<pcl::PointCloud<VPoint>> pc_segs;

        pcl::PointCloud<VPoint>::Ptr laserCloudRect(new pcl::PointCloud<VPoint>());
        remove_outside_rect<VPoint>(laserCloudIn, laserCloudRect, left_distance_,
                                    back_distance_, right_distance_, max_distance_);

        ROS_DEBUG_STREAM(
            "****** laserCloudRect size: " << laserCloudRect->points.size());

        //   pcSegmentX(*laserCloudRect, pc_segs);
        //   for (auto &item : pc_segs)
        {
            // if (item.points.size() > (3 * num_lpr_))
            {
            pcl::PointCloud<VPoint>::Ptr part_no_ground(
                new pcl::PointCloud<VPoint>());
            pcl::PointCloud<VPoint>::Ptr part_ground(new pcl::PointCloud<VPoint>());
            pcl::PointCloud<VPoint> temp;
            //   pcl::copyPointCloud(item, temp);
            //   groundPlaneFitting(item, temp, part_ground, part_no_ground);
            pcl::copyPointCloud(*laserCloudRect, temp);
            groundPlaneFitting(*laserCloudRect, temp, part_ground, part_no_ground);
            (*final_no_ground) += (*part_no_ground);
            (*final_ground) += (*part_ground);
            }
        }
    }
    */
    /**
     * 平面模型估计  用简单的线性模型估计 ax+by+cz+d = 0
     * 点云中的点到这个平面的正交投影距离小于阀值th_dist_d_,则属于地面点
     */
    void PlaneGroudFilter::estimate_plane_() {
    //    Eigen::Matrix3f cov; //初始点集的协方差矩阵，描述种子点集的散布情况
    //    Eigen::Vector4f pc_mean;
    //    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    //    // SVD分解
    //    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    //    normal_ = (svd.matrixU().col(2));  //points*normal_
    //    即为点到平面的正交投影距离
    //
    //    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    //
    //    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    //    th_dist_d_ = th_dist_ - d_; //点到平面的距离阀值

    //    ROS_WARN("D DISTANCHE:{%f}, TH_DIST_D:{%f}",d_, th_dist_d_);
    }

    /**
     * 平面模型估计  用简单的线性模型估计 ax+by+cz+d = 0
     * 点云中的点到这个平面的正交投影距离小于阀值th_dist_d_,则属于地面点
     */
    void PlaneGroudFilter::estimate_plane_(CloudPtr ground_pc) {
        Eigen::Matrix3f cov;  //初始点集的协方差矩阵，描述种子点集的散布情况
        Eigen::Vector4f pc_mean;
        pcl::computeMeanAndCovarianceMatrix(*ground_pc, cov, pc_mean);
        // SVD分解
        JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
        normal_ = (svd.matrixU().col(2));  // points*normal_
                                            // 即为点到平面的正交投影距离

        Eigen::Vector3f seeds_mean = pc_mean.head<3>();

        d_ = -(normal_.transpose() * seeds_mean)(0, 0);
        th_dist_d_ = th_dist_ - d_;  //点到平面的距离阀值

        // ROS_DEBUG("D DISTANCHE:{%f}, TH_DIST_D:{%f}", d_, th_dist_d_);
    }
    /**
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    */
    void PlaneGroudFilter::extract_initial_seeds_(const PointCloudType &p_sorted) {
        // LPR is the mean of low point representative
        double sum = 0;
        int cnt = 0;
        // Calculate the mean height value.
        for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++) {
            sum += p_sorted.points[i].z;
            cnt++;
        }
        double lpr_height = cnt != 0 ? sum / cnt : 0;  // in case divide by 0
        g_seeds_pc->clear();
        // iterate pointcloud, filter those height is less than lpr.height+th_seeds_

        // ROS_DEBUG("LPR HEIGHT:{%f}, cnt:{%f}, p size:{%f}", (float)lpr_height,(float)cnt, (float)p_sorted.points.size());

        for (int i = 0; i < p_sorted.points.size(); i++) {
            if (p_sorted.points[i].z < lpr_height + th_seeds_) {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
            }
        }
    }

    /**
     * 根据高度裁剪掉过高的点云
     * @param in
     * @param out
     */
    void PlaneGroudFilter::clip_above(const CloudPtr in,const CloudPtr out) {
        pcl::ExtractIndices<PointType> cliper;
        cliper.setInputCloud(in);
        pcl::PointIndices indices;
        #pragma omp for
        for (size_t i = 0; i < in->points.size(); i++) {
            if (in->points[i].z > clip_height_) {
            indices.indices.push_back(i);
            }
        }
            cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
            cliper.setNegative(true);
            cliper.filter(*out);
    }

    /**
     * 裁剪掉过近的点云
     * @param in
     * @param out
     */
    void PlaneGroudFilter::remove_close_far_pt(const CloudPtr in,const CloudPtr out) {
        pcl::ExtractIndices<PointType> cliper;

        cliper.setInputCloud(in);
        pcl::PointIndices indices;
        #pragma omp for
        for (size_t i = 0; i < in->points.size(); i++) {
            double distance = sqrt(in->points[i].x * in->points[i].x +
                                in->points[i].y * in->points[i].y);

            if ((distance < min_distance_) || (distance > max_distance_)) {
            indices.indices.push_back(i);
            }
        }
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        cliper.setNegative(true);
        cliper.filter(*out);
    }
    void PlaneGroudFilter::post_process(const CloudPtr in,CloudPtr out) {
        CloudPtr cliped_pc_ptr(new PointCloudType);
        clip_above(in, cliped_pc_ptr);

        CloudPtr remove_close(new PointCloudType);
        remove_close_far_pt(cliped_pc_ptr, out);
    }

    bool point_cmp(const PointType &a, const PointType &b) { return a.z < b.z; }

    bool point_cmp_y(const PointType &a, const PointType &b) { return a.y < b.y; }

    bool point_cmp_x(const PointType &a, const PointType &b) { return a.x < b.x; }

    /**
     * 分割地面
     * @param laserCloudIn
     * @param laserCloudIn_org
     * @param final_ground
     * @param final_no_ground
     */
    void PlaneGroudFilter::groundPlaneFitting(PointCloudType laserCloudIn,PointCloudType laserCloudIn_org,CloudPtr seg_ground,CloudPtr seg_no_ground) {
        //  分割地面点
        SLRPointXYZIRL point;
        CloudPtr g_ground_pc(new PointCloudType());
        CloudPtr g_not_ground_pc(new PointCloudType());
        pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<SLRPointXYZIRL>());
        g_all_pc->clear();
        #pragma omp for
        for (size_t i = 0; i < laserCloudIn.points.size(); i++) {
            point.x = laserCloudIn.points[i].x;
            point.y = laserCloudIn.points[i].y;
            point.z = laserCloudIn.points[i].z;
            point.intensity = laserCloudIn.points[i].intensity;
            //        point.ring = laserCloudIn.points[i].ring;
            point.label = 0u;                   // 0表示未聚类
            g_all_pc->points.push_back(point);  //获取所有的点
        }

        // 按z轴方向距离远近对点进行排序
        sort(laserCloudIn.points.begin(), laserCloudIn.points.end(), point_cmp);
        sort(laserCloudIn_org.points.begin(), laserCloudIn_org.points.end(),point_cmp);

        // 去掉噪点 消除近点反射，与按Z轴排序共同理解
        PointCloudType::iterator it = laserCloudIn.points.begin();
        PointCloudType::iterator it_org = laserCloudIn_org.points.begin();
        // #pragma omp for
        for (int i = 0; i < laserCloudIn.points.size(); i++) {
            if (laserCloudIn.points[i].z < -1.5 * sensor_height_) {   //按照Z值从小到大排序
                it++;
                it_org++;
            } else {
                break;
            }
        }
        laserCloudIn.points.erase(laserCloudIn.points.begin(), it);
        laserCloudIn_org.points.erase(laserCloudIn_org.points.begin(), it_org);

        //  提取初始种子点
        extract_initial_seeds_(laserCloudIn);
        g_ground_pc = g_seeds_pc;

        //  地面平面拟合主循环
        //#pragma omp for
        for (int i = 0; i < num_iter_; i++) {
            //        平面模型估计
            // estimate_plane_();
            estimate_plane_(g_ground_pc);
            g_ground_pc->clear();
            g_not_ground_pc->clear();
            // g_all_pc->clear();

            // pointcloud to matrix
            MatrixXf points(laserCloudIn_org.points.size(), 3);

            int j = 0;
            for (auto &p : laserCloudIn_org.points) {
            points.row(j++) << p.x, p.y, p.z;
            }
            // 点到平面的正交投影距离
            VectorXf result = points * normal_;
            // 距离阀值过滤
            for (int r = 0; r < result.rows(); r++) {
                if (result[r] < th_dist_d_) {  //地面
                    g_all_pc->points[r].label = 1u;
                    g_ground_pc->points.push_back(laserCloudIn_org[r]);
                } else {  //非地面，未聚类
                    g_all_pc->points[r].label = 0u;
                    g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
                }
            }
        }
        *seg_ground = *g_ground_pc;

        //   *seg_no_ground = *g_not_ground_pc;

        // 进一步处理非地面点
        post_process(g_not_ground_pc, seg_no_ground);
    }

    void PlaneGroudFilter::pcSegmentY(PointCloudType in_cloud,std::vector<PointCloudType> &pc_segs) {
        int n_segs = num_seq_;
        // 按z轴方向距离远近对点进行排序
        sort(in_cloud.points.begin(), in_cloud.points.end(), point_cmp_y);

        double ymin = in_cloud.points.at(0).y;
        double ymax = in_cloud.points.at(in_cloud.points.size() - 1).y;

        double y_step = std::round((ymax - ymin) / n_segs);
        //    ROS_INFO_STREAM("****** y_min is : " << ymin);
        //    ROS_INFO_STREAM("****** y_max is : " << ymax);
        //    ROS_INFO_STREAM("****** y_step is : " << y_step);

        for (int i = 0; i < n_segs; ++i) {
            PointCloudType pc_seg;
            for (int k = 0; k < in_cloud.points.size(); ++k) {
            if (in_cloud.points.at(k).y >= ymin + y_step * i &&
                in_cloud.points.at(k).y < ymin + y_step * (i + 1)) {
                pc_seg.push_back(in_cloud.points.at(k));
            }
            }
            pc_segs.push_back(pc_seg);
        }
    }

    void PlaneGroudFilter::pcSegmentX(PointCloudType in_cloud,std::vector<PointCloudType> &pc_segs) {
        int n_segs = num_seq_;
        // 按x轴方向距离远近对点进行排序
        sort(in_cloud.points.begin(), in_cloud.points.end(), point_cmp_x);
        double xmin = in_cloud.points.at(0).x;
        double xmax = in_cloud.points.at(in_cloud.points.size() - 1).x;

        double x_step = std::round((xmax - xmin) / n_segs);
        //    ROS_INFO_STREAM("****** x_min is : " << xmin);
        //    ROS_INFO_STREAM("****** x_max is : " << xmax);
        //    ROS_INFO_STREAM("****** x_step is : " << x_step);

        for (int i = 0; i < n_segs; ++i) {
            PointCloudType pc_seg;
            for (int k = 0; k < in_cloud.points.size(); ++k) {
            if (in_cloud.points.at(k).x >= xmin + x_step * i &&
                in_cloud.points.at(k).x < xmin + x_step * (i + 1)) {
                pc_seg.push_back(in_cloud.points.at(k));
            }
            }
            pc_segs.push_back(pc_seg);
        }
    }


}