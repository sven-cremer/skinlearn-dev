# This is the README file for the force control package

# Isura Ranatunga
# 01/22/2014

# get pached PR2 stacks
git clone https://github.com/PR2/pr2_ethercat_drivers.git -b groovy-devel &&
git clone https://github.com/PR2/pr2_mechanism.git -b groovy-devel &&
git clone https://github.com/PR2/pr2_robot.git -b groovy-devel &&
git clone https://github.com/PR2/pr2_controllers.git -b master &&

# Update bash file
export ROS_PACKAGE_PATH=~/groovy_workspace/sandbox/pr2_mechanism/:$ROS_PACKAGE_PATH 
export ROS_PACKAGE_PATH=~/groovy_workspace/sandbox/pr2_ethercat_drivers/:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=~/groovy_workspace/sandbox/pr2_controllers/:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=~/groovy_workspace/sandbox/pr2_robot/:$ROS_PACKAGE_PATH
