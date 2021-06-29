# ROS_SCAN_MATCHING

rosrun scan_matching localizer

#Python2 & Matplotlib & Numpy
sudo apt-get install python-matplotlib python-numpy python2.7-dev

# The first time to run the node (or if change the paramter in my_init.yaml), need to load parameter 
rosparam load ~/catkin_ws/src/icp_lokalisierung/scan_matching/cfg/my_init.yaml

rosrun scan_mathcing hb2
