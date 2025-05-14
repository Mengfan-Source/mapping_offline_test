cd ./build
bin/test_run_frontend
bin/test_dump_map --pose_source lidar
bin/test_find_loopclosure
bin/test_run_optimization
bin/test_dump_map --pose_source opti2
