#include <gflags/gflags.h>
#include <glog/logging.h>
#include "loopclosure/loopclosure.h"

DEFINE_string(config_yaml, "./../src/config/mapping_mid360.yaml", "配置文件");

int main(int argc, char** argv) {
    FLAGS_log_dir = "./../log";
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    xmf::LoopClosure lc(FLAGS_config_yaml);
    lc.Init();
    lc.Run();

    return 0;
}