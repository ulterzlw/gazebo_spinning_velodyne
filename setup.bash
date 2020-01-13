BASE=$(cd "`dirname "${BASH_SOURCE[0]}"`";pwd)
if [ ! -e $BASE/build ]
then
  mkdir -p $BASE/build
fi
cd $BASE/build
if [ ! -e ./Makefile ]
then
  cmake ..
fi
make
cd - >> /dev/null
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(cd $BASE/build;pwd)
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(cd $BASE/models;pwd)
