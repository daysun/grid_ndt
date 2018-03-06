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
//with color information
int num =0;

pcl::PointCloud<pcl::PointXYZRGB>cloud;

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr & my_msg)
{
    cout<<"receive file\n";
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*my_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    for (int i=0;i<temp_cloud->points.size();i++,num++)
    {
        cloud.points[num].x = temp_cloud->points[i].x;
        cloud.points[num].y = temp_cloud->points[i].y;
        cloud.points[num].z = temp_cloud->points[i].z;
        cloud.points[num].r = temp_cloud->points[i].r;
        cloud.points[num].g = temp_cloud->points[i].g;
        cloud.points[num].b = temp_cloud->points[i].b;
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

  ros::Subscriber sub = n.subscribe("/camera/rgb/points", 1000, chatterCallback);
  ros::spin();
  ros::shutdown();
  pcl::io::savePCDFileASCII("rgbd_dataset.pcd",cloud);
  cout<<"write file\n";
  return 0;
}

