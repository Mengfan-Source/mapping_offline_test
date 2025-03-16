#include <gflags/gflags.h>
#include <glog/logging.h>

#include "frontend/frontend.h"

DEFINE_string(config_yaml, "./../src/config/mapping_mid360.yaml", "配置文件");

int main(int argc, char** argv) {
    FLAGS_log_dir = "./../log";
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;//设置日志输出级别
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);//解析命令行参数
    LOG(INFO) << "testing frontend";
    xmf::Frontend frontend(FLAGS_config_yaml);
    if (!frontend.Init()) {
        LOG(ERROR) << "failed to init frontend.";
        return -1;
    }

    frontend.Run();
    return 0;
}