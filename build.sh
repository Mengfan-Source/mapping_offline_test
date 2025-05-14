cp mapping_offine.sh ./../../../../CmakeBuild/scripts/
build_dir=$(pwd)/build
rm -rf ${build_dir}
mkdir ${build_dir}
cd ${build_dir}
cmake ..
make -j8
rm -rf ${build_dir}
