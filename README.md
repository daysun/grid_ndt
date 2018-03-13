grid_ndt
===========
<b>Dependencies</b><br/>
1、OpenCV(http://docs.opencv.org/master/d9/df8/tutorial_root.html)<br/>
2、PCL(http://pointclouds.org/downloads/)<br/>
3、OpenMP(http://www.openmp.org/)<br/>
4、rviz(http://wiki.ros.org/rviz)<br/>


<b>Compile</b><br/>
You should put this project under your ROS work space, such as <i>your_ros_path/src/</i><br/>
<i>cd your_ros_path</i><br/>
<i>catkin_make</i><br/>

<b>Usage</b><br/>
1、 <i>roslaunch grid_ndt receLaunch.launch</i><br/>
2、 <i>roslaunch grid_ndt ocLaunch.launch</i><br/>
&ensp;&ensp;The experimental data (*.pcd) can be attained in https://pan.baidu.com/s/1V3h0yNY5YEQUvvzE06LdWQ,
and the corresponding parameters can be attained in /launch/parameters.txt.<br/>
For visualization, the rviz should subscribe the top "initial_marker_array", "traversibility_marker_array" and "route_marker_array".


