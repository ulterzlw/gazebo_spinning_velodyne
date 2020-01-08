BASE=$(cd $(dirname $0);pwd)
cd $BASE
mkdir build
cd build
cmake ..
make
cd -
cp -r models ~/.gazebo
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(cd ./build;pwd)
