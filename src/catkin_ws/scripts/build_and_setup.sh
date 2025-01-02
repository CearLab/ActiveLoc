#!/bin/bash

# Run catkin make
catkin_make

# Check if catkin make was successful
if [ $? -eq 0 ]; then
    echo "catkin make successful"
else
    echo "catkin make failed"
    exit 1
fi

# Run setup.bash
source devel/setup.bash

# Check if setup.bash was sourced successfully
if [ $? -eq 0 ]; then
    echo "setup.bash sourced successfully"
else
    echo "Failed to source setup.bash"
    exit 1
fi



# Add any additional commands or actions you want to perform after sourcing setup.bash

# Example: Run your program
# roslaunch my_package my_launch_file.launch