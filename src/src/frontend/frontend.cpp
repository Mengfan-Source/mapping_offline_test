#include "frontend/frontend.h"
#include "lioiekf/lio_iekf.h"
#include "io_utils/io_utils.h"

#include <yaml-cpp/yaml.h>

namespace xmf {

Frontend::Frontend(const std::string& config_yaml) { config_yaml_ = config_yaml; }

bool Frontend::Init() {
    pose_tum = std::ofstream("./../data/evo/frontend.txt",std::ios::trunc);

    LOG(INFO) << "load yaml from " << config_yaml_;
    auto yaml = YAML::LoadFile(config_yaml_);
    try {
        auto n = yaml["bag_path"];
        LOG(INFO) << Dump(n);
        bag_path_ = yaml["bag_path"].as<std::string>();
        lio_yaml_ = yaml["lio_yaml"].as<std::string>();
        map_path_ = yaml["map_data"].as<std::string>();
        bag_type_ = yaml["bag_type"].as<std::string>();
    } catch (...) {
        LOG(ERROR) << "failed to parse yaml";
        return false;
    }

    system((std::string("rm -rf ")+map_path_+std::string("*.pcd")).c_str());
    system((std::string("rm -rf ")+map_path_+std::string("keyframes.txt")).c_str());
    
    LioIEKF::Options options;
    options.with_ui_ = false;  // 跑建图不需要打开前端UI
    lio_ = std::make_shared<LioIEKF>(options);
    lio_->Init(lio_yaml_);
    return true;
}

void Frontend::Run() {
    std::shared_ptr<xmf::RosbagIO> rosbag_io = nullptr;
    if(bag_type_ == "AIRY")
        // xmf::RosbagIO rosbag_io(bag_path_, DatasetType::AIRY);
        rosbag_io = std::make_shared<xmf::RosbagIO>(bag_path_, DatasetType::AIRY);
    else if(bag_type_ == "LIVOXMID360")
        // xmf::RosbagIO rosbag_io(bag_path_, DatasetType::LIVOXMID360);
        rosbag_io = std::make_shared<xmf::RosbagIO>(bag_path_, DatasetType::LIVOXMID360);

    if(rosbag_io->dataset_type_ == DatasetType::NCLT){
        // 先提取RTK pose，注意NCLT只有平移部分
        rosbag_io
            ->AddAutoRTKHandle([this](GNSSPtr gnss) {
                gnss_.emplace(gnss->unix_time_, gnss);
                return true;
            })
            .Go();
        rosbag_io->CleanProcessFunc();  // 不再需要处理RTK
        RemoveMapOrigin();
    }

    // 再运行LIO
    rosbag_io
        ->AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr cloud) -> bool {
            lio_->PCLCallBack(cloud);
            ExtractKeyFrame(lio_->GetCurrentState());
            return true;
        }).AddLivoxHandle([&](const livox_ros_driver2::CustomMsg::ConstPtr &msg) -> bool {
            lio_->LivoxPCLCallBack(msg);
            ExtractKeyFrame(lio_->GetCurrentState());
            // 保存tum格式的pose
            xmf::NavState<double> temp_state = lio_->GetCurrentState();
            auto save_SE3 = [](std::ostream &f, SE3 pose) {
            auto q = pose.so3().unit_quaternion();
            Vec3d t = pose.translation();
            f << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();};
            pose_tum <<std::setprecision(18) << temp_state.timestamp_<<" ";
            save_SE3(pose_tum,temp_state.GetSE3());
            pose_tum<<std::endl;
            return true;
        }).AddImuHandle([&](IMUPtr imu) {
            lio_->IMUCallBack(imu);
            return true;
        })
        .Go();
    lio_->Finish();

    // 保存运行结果
    SaveKeyframes();

    LOG(INFO) << "done.";
    pose_tum.close();
}

void Frontend::ExtractKeyFrame(const xmf::NavStated& state) {
    if (last_kf_ == nullptr) {
        if (!lio_->GetCurrentScan()) {
            // LIO没完成初始化
            return;
        }
        // 第一个帧
        auto kf = std::make_shared<Keyframe>(state.timestamp_, kf_id_++, state.GetSE3(), lio_->GetCurrentScan());
        // 通过差值计算该关键帧的GPS位置
        FindGPSPose(kf);
        kf->SaveAndUnloadScan(map_path_);
        keyframes_.emplace(kf->id_, kf);
        last_kf_ = kf;
    } else {
        // 计算当前state与kf之间的相对运动阈值
        SE3 delta = last_kf_->lidar_pose_.inverse() * state.GetSE3();
        if (delta.translation().norm() > kf_dis_th_ || delta.so3().log().norm() > kf_ang_th_deg_ * math::kDEG2RAD) {
            auto kf = std::make_shared<Keyframe>(state.timestamp_, kf_id_++, state.GetSE3(), lio_->GetCurrentScan());
            FindGPSPose(kf);
            keyframes_.emplace(kf->id_, kf);
            kf->SaveAndUnloadScan(map_path_);
            LOG(INFO) << "生成关键帧" << kf->id_;
            last_kf_ = kf;
        }
    }
}

void Frontend::FindGPSPose(std::shared_ptr<Keyframe> kf) {
    if(0)
    {
        SE3 pose;
        GNSSPtr match;
        if (math::PoseInterp<GNSSPtr>(
                kf->timestamp_, gnss_, [](const GNSSPtr& gnss) -> SE3 { return gnss->utm_pose_; }, pose, match)) {
            kf->rtk_pose_ = pose;
            kf->rtk_valid_ = true;
        } else {
            kf->rtk_valid_ = false;
        }
    }
    else{
        SE3 pose;
        GNSSPtr match;
        kf->rtk_pose_ = pose;
        kf->rtk_valid_ = false;
    }

}

void Frontend::SaveKeyframes() {
    // std::ofstream fout("./../data/mapdata/keyframes.txt");
     std::ofstream fout((map_path_+std::string("keyframes.txt")).c_str());
    for (auto& kfp : keyframes_) {
        kfp.second->Save(fout);
    }
    fout.close();
}

void Frontend::RemoveMapOrigin() {
    if (gnss_.empty()) {
        return;
    }

    bool origin_set = false;
    for (auto& p : gnss_) {
        if (p.second->status_ == GpsStatusType::GNSS_FIXED_SOLUTION) {
            map_origin_ = p.second->utm_pose_.translation();
            origin_set = true;

            LOG(INFO) << "map origin is set to " << map_origin_.transpose();

            auto yaml = YAML::LoadFile(config_yaml_);
            std::vector<double> ori{map_origin_[0], map_origin_[1], map_origin_[2]};
            yaml["origin"] = ori;

            std::ofstream fout(config_yaml_);
            fout << yaml;
            break;
        }
    }

    if (origin_set) {
        LOG(INFO) << "removing origin from rtk";
        for (auto& p : gnss_) {
            p.second->utm_pose_.translation() -= map_origin_;
        }
    }
}

}  // namespace xmf