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
#include <unistd.h>
using namespace std;

//area of these 4 pcd files
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
  float minx=cloud->points[0].x,miny=cloud->points[0].y,minz=cloud->points[0].z,
          maxx=cloud->points[0].x,maxy=cloud->points[0].y,maxz=cloud->points[0].z;
  for(int i =1;i<cloud->points.size();i++){
      if(cloud->points[i].x >maxx)
          maxx = cloud->points[i].x;
      if(cloud->points[i].x <minx)
          minx = cloud->points[i].x;
      if(cloud->points[i].y >maxy)
          maxy = cloud->points[i].y;
      if(cloud->points[i].y <miny)
          miny = cloud->points[i].y;
      if(cloud->points[i].z >maxz)
          maxz = cloud->points[i].z;
      if(cloud->points[i].z <minz)
          minz = cloud->points[i].z;
  }
  cout<<"x "<<maxx-minx<<endl<<"y "<<maxy-miny<<endl<<"z "<<maxz-minz<<endl;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
//  ros::NodeHandle n;
//  ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud2>("publisher/cloud_fullSys", 1000);
//   sensor_msgs::PointCloud2 output;
   std::string cloud_path("/home/daysun/rros/src/grid_ndt/data/freiburg_23tri.pcd");
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

   if (!loadCloud(cloud_path,cloud))
     return -1;

//   pcl::toROSMsg(*cloud,output);

//   ros::Rate r(30);
//   if(ros::ok())
//       if(chatter_pub.getNumSubscribers() == 1) {
//            chatter_pub.publish(output);
//            cout<<"send out\n";
//            ros::spinOnce();
//            r.sleep();
//       }
  return 1;
}
