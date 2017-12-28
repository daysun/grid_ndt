#include "ros/ros.h"
#include "std_msgs/String.h"
#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <sstream>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <limits.h>
using namespace std;

bool loadCloud(std::string &filename,pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  std::cout << "Loading file " << filename.c_str() << std::endl;
  //read cloud
  if (pcl::io::loadPCDFile(filename, *cloud))
  {
    std::cout << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
    return false;
  }
  //remove NaN Points
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;
  //remove irrelevant points-outlier
//  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (cloud);
//    sor.setMeanK (50);
//    sor.setStddevMulThresh (1.5);
//    sor.filter (*cloud);
//    std::cerr << "Cloud after filtering: " << cloud->points.size()<<std::endl;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud2>("publisher/cloud_del", 1000);
   sensor_msgs::PointCloud2 output;
   std::string cloud_path("/home/daysun/rros/src/grid_ndt/data/delete.pcd");
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

   if (!loadCloud(cloud_path,cloud))
     return -1;

   pcl::toROSMsg(*cloud,output);

   ros::Rate r(30);
   if(ros::ok())
       if(chatter_pub.getNumSubscribers()) {
            chatter_pub.publish(output);
            cout<<"send out\n";
            ros::spinOnce();
            r.sleep();
       }
  return 0;
}
