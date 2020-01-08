BASE=$(cd "`dirname "${BASH_SOURCE[0]}"`";pwd)
cd $BASE
mkdir build
cd build
cmake ..
make
cp -r $BASE/models ~/.gazebo
echo $(cd $BASE/build;pwd)
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(cd $BASE/build;pwd)
cd -
