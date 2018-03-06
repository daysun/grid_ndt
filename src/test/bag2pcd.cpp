#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <assert.h>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include<pcl/io/pcd_io.h>

#include <vector>
#include <fstream>
#include<sstream>
using namespace Eigen;
using namespace std;

//receive the data from the rosbag and turn it into pcd file
int num =0;
//string intToString(int k){
//    stringstream ss;
//   ss<<k;
//   string str = ss.str();
//   return str;
//}

pcl::PointCloud<pcl::PointXYZ>cloud;

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr & my_msg)
{
    cout<<"receive\n";
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*my_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    for (int i=0;i<temp_cloud->points.size();i++,num++)
    {
        cloud.points[num].x = temp_cloud->points[i].x;
        cloud.points[num].y = temp_cloud->points[i].y;
        cloud.points[num].z = temp_cloud->points[i].z;
    }
    cout<<"out\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bag2pcd");
  ros::start();
  ros::NodeHandle n;

  cloud.width = 400;
  cloud.height = 4000;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width*cloud.height);

//  ros::Subscriber sub = n.subscribe("/kitti/velo/pointcloud", 1000, chatterCallback);
  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, chatterCallback);
  ros::spin();
  ros::shutdown();
  pcl::io::savePCDFileASCII("rgbd_pointcloud.pcd",cloud);
  cout<<"write file\n";
  return 0;
}

