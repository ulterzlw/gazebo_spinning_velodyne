BASE=$(cd "`dirname "${BASH_SOURCE[0]}"`";pwd)
cd $BASE
mkdir $BASE/build
cd $BASE/build
cmake ..
make
cd -
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(cd $BASE/build;pwd)
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(cd $BASE/models;pwd)