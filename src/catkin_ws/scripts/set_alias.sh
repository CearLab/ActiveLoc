#!/bin/bash
alias setupcatkin='source /home/idosh/ActiveLoc/src/catkin_ws/devel/setup.bash'
alias bringup="roslaunch jackal_custom bringup.launch"
alias gcs="roslaunch jackal_custom GCS.launch"
# alias move="roslaunch jackal_move follow_goal.launch"
move() {
    roslaunch jackal_move follow_goal.launch X:=$1 Y:=$2
}
setupcatkin
echo "OK"