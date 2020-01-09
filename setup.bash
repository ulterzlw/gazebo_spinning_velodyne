BASE=$(cd "`dirname "${BASH_SOURCE[0]}"`";pwd)
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(cd $BASE/build;pwd)
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(cd $BASE/models;pwd)