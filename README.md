# 1. MAPPING OFFLINE TEST（离线建图算法）
# 2. 算法架构：前端IESKF+NDT，后端因子图优化
# 3. 部署流程
## 1. 下载代码并创建工程文件夹
```bash
git clone https://github.com/Mengfan-Source/mapping_offline_test.git
#针对airy激光雷达,切换到airy分支再进行编译，或者按照更新日志20250320进行两处代码修改也可
git switch airy
#针对X30(揽沃数据格式),直接编译运行即可
cd mapping_offline_test
mkdir -p data/evo
mkdir -p data/mapdata
mkdir -p data/tempout
mkdir log
mkdir build
```
## 2. 安装依赖
```bash
sudo apt install -y ros-noetic-pcl-ros ros-noetic-velodyne-msgs libopencv-dev libgoogle-glog-dev libeigen3-dev libsuitesparse-dev libpcl-dev libyaml-cpp-dev libbtbb-dev libgmock-dev
``` 
## 3. 通过cmake, make安装包里面自带的thirdparty/g2o库
```bash 
cd src/thirdparty/g2o
mkdir build
cd build
cmake ..
make
sudo make install#如果之前安装过g2o就不需要安装了
``` 
## 4. 开始整体工程编译并运行
```bash 
mkdir build
cd build
cmake ..
make -j8
#执行在build目录下执行
bin/test_frontend
``` 
# 5. 跑图流程：
``` bash
#适配数据集：
#1.对于LIVOX和非LIVOXIMU要在/src/src/io_utils/io_utils.cpp的第68行修改IMU量纲
#2.修改激光雷达和IMU话题名称：/src/src/common/dataset_type.h的第25行修改
#配置参数：
    #在src/config/mapping_mid360.yaml文件中修改
        #bag_path
        #lio_yaml
        #map_data
        #三个路径，其中map_data在程序中写死了，不需要设置
#3.编译
cd /home/cetc21/xmf/my_slam_ws/mapping_offline_test/build
cmake ..
make -j8
#4.运行LIO
cd /home/cetc21/xmf/my_slam_ws/mapping_offline_test/build
bin/test_run_frontend
#该步骤执行里程计算法，会在/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/mapdata文件夹下保存运动畸变去除后的关键帧激光雷达数据（未确定是Lidar系下还是IMU系下的）关键帧是通过欧氏距离筛选的，所以比较稀疏
#该步骤还会在该目录下生成关键帧的pose数据，用于点云帧拼接、寻找回环、图优化等步骤
#该步骤还会在/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/evo文件夹下生成用于evo评测的数据（这个tum数据不是关键帧每一帧计算结果都会出来），评测LIO步骤（与FAST-LIO对比）：
cd /home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/evo
evo_traj tum frontend.txt --ref fast_lio_tum_withwaican.txt -p --plot_mode=xy #记得删除前100行的000（这是IMU初始化时候的数据）
evo_ape tum fast_lio_tum_withwaican.txt frontend.txt --plot --plot_mode xyz
#5.根据LIO的位姿拼接点云
bin/test_dump_map --pose_source lidar#默认参数是lidar所以这一步拼接点云可以不带lidar参数
#该步骤会根据lio生成的pose在/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/mapdata下生成map_lidar.pcd
#6.运行寻找回环算法
cd /home/cetc21/xmf/my_slam_ws/mapping_offline_test/build
bin/test_find_loopclosure
#该步骤会寻找并计算回环对，并保存在/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/mapdata/loops.txt中
#7.运行后端因子图优化算法
cd /home/cetc21/xmf/my_slam_ws/mapping_offline_test/build
bin/test_run_optimization
#该步骤会构建因子图优化算法，将优化的位姿也保存在/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/mapdata/keyframes.txt文件中，与LIO并存
#同时还会在/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/evo文件夹下生成lio_keframes.txt和loopclosure_keframes.txt分别存储回环前和回环后的关键帧tum格式的位姿，用作评测
#8.根据回环检测后的位姿拼接点云
cd /home/cetc21/xmf/my_slam_ws/mapping_offline_test/build
bin/test_dump_map --pose_source opti2
#该步骤会根据回环检测后的位姿在/home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/mapdata下生成map_opti2.pcd
#9.evo评测
cd /home/cetc21/xmf/my_slam_ws/mapping_offline_test/data/evo
evo_traj tum frontend.txt lio_keframes.txt loopclosure_keframes.txt --ref fast_lio_tum_withwaican.txt -p --plot_mode=xy
#10.有无回环的建图效果如下所示
``` 
# 6. 开发过程文档
飞书文档：1126-1206
[Company20241120](https://uw7f7qxdyrb.feishu.cn/docx/PGNYd6jNIox4i7xUY9jcgUR4nIf#share-VfLWdVhAxomYjpxf3qLcxDBCn3c)
# 7. 更新日志
- 20250320
    - 适配了airy激光雷达：在各个部分添加了适配airy激光雷达的代码
        - 1.添加了airy激光雷达自带的IMU处理，在代码中量纲处理和坐标变换，所以外参设置为单位矩阵即可
        - 2.添加了airy激光雷达数据类型的支持
        - 3.一帧激光雷达中每个点偏移时间与其他雷达不同，接入使用IMU数据进行激光雷达运动畸变去除的SLAM算法时，需要计算偏移量（减去第一点的偏移时间即可）
            一帧数据中每个点的偏移时间与其他雷达不同，rs_lidar的偏移时间是运行时刻递增的，而不是一个单纯的偏移量，因此在接入使用IMU进行运动畸变去除的SLAM算法时需要将每个点的偏移时间设置为当前点的时间减去第一个点的相对时间。
        - 4.调整NDT配准参数
        对于airy激光雷达，猜测由于重复扫描方式，NDT配准过程中的有效点数量较少，修改mapping_ofline_test/src/src/ndt_inc.cpp:288行这个参数改小后再airy雷达上能跑了,原参数（X30数据）0.01，现在0.00001（0.0001也能跑，但是效果不是很好）可以跑
    - 建立了针对airy雷达（airy自身IMU）的yaml文件，并新建或修改如下参数
        - 新建文件:mapping_airy.yaml lio_airy.yaml
        - 参数bag_type可选配置：LIVOXMID360(揽沃格式雷达数据，IMU是标准格式)  AIRY(pcl2格式雷达数据,IMU量纲是1，且方向和airy外参一致)
        - 针对airy雷达除了外参需要修改，还需要设置time_scale为1000，因为运动畸变去除代码中需要的偏移时间是ms，airy雷达数据格式的偏移时间是s
        - 针对揽沃雷达数据格式这个调整值是写死在代码中的 /1000000，猜测揽沃雷达偏移时间单位是纳秒
        - 针对airy激光雷达point_filter_num:设置为2为宜（airy雷达一帧中有8w个点）
    - 建立airy分支，并将适配了airy直接能跑的代码上传到airy分支
        - airy分支和main分支其他都相同，针对airy分支的代码也都同步改动
        - airy分支和mian分支区别只有两点：
            - mapping_ofline_test/src/src/ndt_inc.cpp:288在airy中为0.00001（0.0001也能跑，但是效果不是很好），在main分支为0.01（针对X30数据）
            - mapping_offline_test/src/test/run_frontend.cpp:6 中yaml配置文件替换
        - 如果不切换分支按照这条修改这两个部分也可直接编译运行。
