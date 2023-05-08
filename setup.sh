#!/bin/sh
SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  WS_DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
  SOURCE=$(readlink "$SOURCE")
  [[ $SOURCE != /* ]] && SOURCE=$DIR/$SOURCE # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
export WS_DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )

echo "WS DIR: "$WS_DIR

echo -n "ROS setup... " 
source devel/setup.bash
echo "DONE"

echo -n "Python setup... "
export PYTHONPATH=$PYTHONPATH:$WS_DIR
echo "DONE"

export TURTLEBOT_MODEL=turtlebot3_waffle_pi
printf "Turtlebot model name: $TURTLEBOT_MODEL \n"

# echo
# echo "PYTHONPATH: "$PYTHONPATH