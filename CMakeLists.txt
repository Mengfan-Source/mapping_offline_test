cmake_minimum_required(VERSION 3.16)
project(xmf_mapping_offline_test VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17) 

set(CMAKE_BUILD_TYPE "Release")#使用release版本可以提高程序的运行效率
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -ggdb")#-O3 这是一个编译器优化标志，指示编译器应用最高级别的优化以提高程序的运行速度。
                                                                                        #-ggdb：这是一个 GCC 特定的编译器标志，用于指定使用 GDB 作为调试器，并生成与 GDB 兼容的调试信息。

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  pcl_conversions
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpart 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_msgs   
        ${catkin_INCLUDE_DIRS} 
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}

        ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpart/g2o/
)
set(g2o_libs
        ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpart/g2o/lib/libg2o_stuff.so
        ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpart/g2o/lib/libg2o_core.so
	# ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_cholmod.so
        ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpart/g2o/lib/libg2o_solver_dense.so
        ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpart/g2o/lib/libg2o_solver_csparse.so
        ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpart/g2o/lib/libg2o_solver_csparse.so
        ${CMAKE_CURRENT_SOURCE_DIR}/src/thirdpart/g2o/lib/libg2o_types_sba.so
)
#生成静态库版本：执行效率高
add_library(
        LIO_STATICLIB_ALL
        STATIC
        src/src/frontend/frontend.cpp
        src/src/frontend/keyframe.cpp
        src/src/common/point_cloud_utils.cpp
        src/src/lioiekf/lio_iekf.cpp
        src/src/lioiekf/static_imu_init.cpp
        src/src/lioiekf/measure_sync.cpp
        src/src/ndt_inc/ndt_inc.cpp
        src/src/common/g2o_types.cpp
        src/src/common/timer.cpp
        src/src/imu_preintegration/imu_preintegration.cpp

        src/src/io_utils/io_utils.cpp
        src/src/sensors/utm_convert.cpp
        src/thirdpart/utm_convert/utm.cc
        src/thirdpart/utm_convert/tranmerc.cc
        src/src/common/global_flags.cpp
        src/src/sensors/cloud_convert.cpp
)
add_executable(
        test_run_frontend
        src/test/run_frontend.cpp
)
target_link_libraries(
        test_run_frontend
        # PUBLIC
        LIO_STATICLIB_ALL
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${yaml-cpp_LIBRARIES}
        yaml-cpp
        tbb
        glog
        gflags  
        ${g2o_libs}
)
#点云合并
add_executable(
    test_dump_map
    src/test/dump_map.cpp
)
target_link_libraries(
    test_dump_map
    LIO_STATICLIB_ALL
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    glog
    gflags 
)
#查找回环对
add_library(
        FINDLOOPCLOSURE_STATICLIB_ALL
        STATIC
        src/src/loopclosure/loopclosure.cpp
        src/src/frontend/keyframe.cpp
        src/src/common/point_cloud_utils.cpp
)
add_executable(
    test_find_loopclosure
    src/test/run_find_loopclosure.cpp
)
target_link_libraries(
    test_find_loopclosure
    FINDLOOPCLOSURE_STATICLIB_ALL
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    yaml-cpp
    glog
    gflags
    tbb
)
#回环因子图优化
add_library(
        OPTIMIZATION_STATICLIB_ALL
        STATIC
        src/src/optimization/optimization.cpp
        src/src/loopclosure/loopclosure.cpp
        src/src/frontend/keyframe.cpp
        src/src/common/point_cloud_utils.cpp  
)
add_executable(
    test_run_optimization
    src/test/run_optimization.cpp
)
target_link_libraries(
    test_run_optimization
    OPTIMIZATION_STATICLIB_ALL
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
    yaml-cpp
    glog
    gflags
    tbb
    ${g2o_libs}
)

#生成静态库之前的版本
# add_executable(test_run_frontend 
#         src/test/run_frontend.cpp
#         src/src/frontend/frontend.cpp
#         src/src/frontend/keyframe.cpp
#         src/src/common/point_cloud_utils.cpp
#         src/src/lioiekf/lio_iekf.cpp
#         src/src/lioiekf/static_imu_init.cpp
#         src/src/lioiekf/measure_sync.cpp
#         src/src/ndt_inc/ndt_inc.cpp
#         src/src/common/g2o_types.cpp
#         src/src/common/timer.cpp
#         src/src/imu_preintegration/imu_preintegration.cpp

#         src/src/io_utils/io_utils.cpp
#         src/src/sensors/utm_convert.cpp
#         src/thirdpart/utm_convert/utm.cc
#         src/thirdpart/utm_convert/tranmerc.cc
#         src/src/common/global_flags.cpp
#         src/src/sensors/cloud_convert.cpp
# )
# target_link_libraries(
#         test_run_frontend
#         PUBLIC
#         ${catkin_LIBRARIES}
#         ${EIGEN3_LIBRARIES}
#         ${PCL_LIBRARIES}
#         ${Boost_LIBRARIES}
#         ${OpenCV_LIBRARIES}
#         ${yaml-cpp_LIBRARIES}
#         yaml-cpp
#         tbb
#         glog
#         gflags  
        
#         ${g2o_libs}
# )