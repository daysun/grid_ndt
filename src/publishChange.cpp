#include "ros/ros.h"
#include "std_msgs/String.h"
#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <sstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.5);
    sor.filter (*cloud);
    std::cerr << "Cloud after filtering: " << cloud->points.size()<<std::endl;

    //present as red
    for(size_t i=0;i<cloud->points.size();i++){
//        cloud->points[i].x += 5;
        cloud->points[i].y -= 1;
        cloud->points[i].z += 3;
    }
//    pcl::PCDWriter writer;
//    writer.write<pcl::PointXYZ> ("table_filter2.pcd", *cloud, false);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud2>("publisher/cloud_change", 1000);
   sensor_msgs::PointCloud2 output;
   std::string cloud_path("/home/daysun/rros/src/grid_ndt/data/test.pcd");
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

   if (!loadCloud(cloud_path,cloud))
     return -1;

   pcl::toROSMsg(*cloud,output);
    output.header.frame_id = "/my_frame";
    cout<<"sub num "<<chatter_pub.getNumSubscribers()<<endl;

    ros::Rate r(30);
    if(ros::ok())
        if(chatter_pub.getNumSubscribers() == 1) {
             chatter_pub.publish(output);
             cout<<"send out\n";
             ros::spinOnce();
             r.sleep();
        }

  return 0;
}

