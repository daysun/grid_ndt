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
//#include "map2D.h"
//#include "AstarPlanar.h"
#include "GlobalPlan.h"
#include <vector>
#include <fstream>
#include"omp.h"
using namespace Eigen;
using namespace std;
using namespace daysun;
typedef multimap<string,daysun::OcNode *>  MAP_INT_MORTON_MULTI;
typedef multimap<string,daysun::OcNode *>::iterator iterIntNode;

RobotSphere robot(0.5); //radius--variable--according to the range of map
//test-0.25, sys-0.125 vision-1.5 bag-1
daysun::TwoDmap map2D(0.5);
ros::Publisher marker_pub,change_pub,markerArray_pub,markerArray_pub2,marker_pub_bo,route_pub/*,del_pub*/;
string demand;

//ofstream outfile("/home/daysun/testPointsSys.txt", ofstream::app);

void uniformDivision( const pcl::PointXYZ temp,bool change){
    string morton_xy,morton_z;
    Vec3 temp3(temp.x,temp.y,temp.z);
    map2D.transMortonXYZ(temp3,morton_xy,morton_z);
    //for change-record which xy have been changed
    if(change){
        if(map2D.changeMorton_list.size() != 0){
            list<string>::iterator it = find(map2D.changeMorton_list.begin(), map2D.changeMorton_list.end(), morton_xy);
            if (it == map2D.changeMorton_list.end()){ //not find
                map2D.changeMorton_list.push_back(morton_xy);
            }
        }else{
            map2D.changeMorton_list.push_back(morton_xy);
        }
//        cout<<"change morton: "<<morton_xy<<","<<morton_z<<endl;
    }

    if(map2D.map_xy.count(morton_xy) == 0){ //not found
        //add new node
        daysun::OcNode * node = new daysun::OcNode();
        node->lPoints.push_back(temp);
        node->morton = morton_xy;
        node->z = morton_z;
        map2D.map_xy.insert(MAP_INT_MORTON_MULTI::value_type(morton_xy,node));
//        map2D.map_z.insert(make_pair(morton_z,node));
        map2D.morton_list.push_back(morton_xy);       
    }else{//find        
                    iterIntNode beg = map2D.map_xy.lower_bound(morton_xy);
                     iterIntNode end = map2D.map_xy.upper_bound(morton_xy);
                     bool found = false;
                     while(beg != end){
                         if( (beg->second)->z.compare(morton_z) == 0 ){//string z
                             (beg->second)->lPoints.push_back(temp);
                             found = true;
                             break;
                         }
                         ++beg;
                     }
                     if(!found){
                         daysun::OcNode * node = new daysun::OcNode();
                         node->lPoints.push_back(temp);
                         node->morton = morton_xy;
                         node->z = morton_z;
                         map2D.map_xy.insert(MAP_INT_MORTON_MULTI::value_type(morton_xy,node));
//                         map2D.map_z.insert(make_pair(morton_z,node));
                     }
    }
}

///no-use
///maybe wrong
void uniformDelDivision(const pcl::PointXYZ temp){
    string morton_xy,morton_z;
    Vec3 temp3(temp.x,temp.y,temp.z);
    map2D.transMortonXYZ(temp3,morton_xy,morton_z);

    if(map2D.map_xy.count(morton_xy) == 0){ //not found
//       cout<<"delete data not in the map\n";
        //because of the data, just ignore them
    }else{//find
                    iterIntNode beg = map2D.map_xy.lower_bound(morton_xy);
                     iterIntNode end = map2D.map_xy.upper_bound(morton_xy);
                     bool found = false;
                     while(beg != end){
                         if( (beg->second)->z.compare(morton_z) == 0 ){//string z
                             (beg->second)->lDelPoints.push_back(temp);
                             found = true;
                             break;
                         }
                         ++beg;
                     }
                     if(!found){
                         cout<<"delete-data error\n";
                         return;
                     }

                     //for delete-record which xy have been changed
                         if(map2D.delMorton_list.size() != 0){
                             list<string>::iterator it = find(map2D.delMorton_list.begin(), map2D.delMorton_list.end(), morton_xy);
                             if (it == map2D.delMorton_list.end()){ //not find
                                 map2D.delMorton_list.push_back(morton_xy);
                             }
                         }else{
                             map2D.delMorton_list.push_back(morton_xy);
                         }
    }
}

///initial-one time
void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr & my_msg)
{
    cout<<"---------receive initial---------\n";    
    double time_start = stopwatch();
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*my_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    cout<<temp_cloud->points.size()<<endl;
    map2D.setCloudFirst(Vec3(temp_cloud->points[0].x,temp_cloud->points[0].y,temp_cloud->points[0].z));

    #pragma omp parallel for
    for (int i=1;i<temp_cloud->points.size();i++)
    {
//        outfile<<temp_cloud->points[i].x<<","<<temp_cloud->points[i].y<<","<<temp_cloud->points[i].z<<endl;
        uniformDivision(temp_cloud->points[i],false);
     }
     double time_end = stopwatch();
//    outfile.close();
    cout<<"division time: "<<(time_end-time_start)<<" s\n";
    cout<<"grid length "<<map2D.getGridLen()<<", morton size: "<<map2D.morton_list.size()<<endl;

    double time_start1 = stopwatch();
    map2D.create2DMap(demand);
    double time_end1 = stopwatch();
    cout<<"calculate time: "<<(time_end1-time_start1)<<" s\n";

   if(marker_pub.getNumSubscribers()){
        map2D.showInital(marker_pub,robot,0);
//        map2D.showBottom(marker_pub_bo);
        cout<<"initial show done\n";
    }    

    Vec3 goal(robot.getGoal());
    map2D.computeCost(goal,robot,markerArray_pub,markerArray_pub2,demand); //compute the cost map

    AstarPlanar globalPlanr(robot.getPosition(),robot.getGoal());
    if(globalPlanr.findRoute(map2D,robot,demand) && route_pub.getNumSubscribers())
        globalPlanr.showRoute(map2D,route_pub);
}

///change points-many times
void changeCallback(const sensor_msgs::PointCloud2::ConstPtr & my_msg){
     cout<<"---------receiveChange---------\n";
     double time_start1 = stopwatch();
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*my_msg,pcl_pc2);
     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

     map2D.changeMorton_list.clear();
     #pragma omp parallel for
     for (int i=0;i<temp_cloud->points.size();i++)
     {
         uniformDivision(temp_cloud->points[i],true);
      }
     double time_end1 = stopwatch();
     cout<<"point size "<<temp_cloud->points.size()<<endl;
     cout<<"change/add map- division done. Time cost: "<<(time_end1-time_start1)<<" s\n";
     cout<<"change/add map- morton size: "<<map2D.changeMorton_list.size()<<endl;

     double time_start2 = stopwatch();
     map2D.change2DMap();
     double time_end2 = stopwatch();
     cout<<"change/add map- 2D Map creation done. Time cost: "<<(time_end2-time_start2)<<" s\n";

     if(change_pub.getNumSubscribers()){
         map2D.showInital(change_pub,robot,1);
         cout<<"change/add map- show done\n";
     }
//          if(marker_pub.getNumSubscribers()){
//              map2D.showInital(marker_pub,robot,1);
//              cout<<"change/add map- show done\n";
//          }
     //plan-not yet
}

///delete points-many times
///the same procedure as initial
void delCallback(const sensor_msgs::PointCloud2::ConstPtr & my_msg){
     cout<<"---------receive Delete points---------\n";
     double time_start1 = stopwatch();
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*my_msg,pcl_pc2);
     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
     cout<<temp_cloud->points.size()<<endl;

     map2D.delMorton_list.clear();
     cout<<"point size "<<temp_cloud->points.size()<<endl;
     #pragma omp parallel for
     for (int i=0;i<temp_cloud->points.size();i++)
     {
         uniformDelDivision(temp_cloud->points[i]);
      }
     double time_end1 = stopwatch();
     cout<<"delete map- division done. Time cost: "<<(time_end1-time_start1)<<" s\n";
     cout<<"delete map- morton size: "<<map2D.delMorton_list.size()<<endl;

     double time_start2 = stopwatch();
     map2D.del2DMap();
     double time_end2 = stopwatch();
     cout<<"change/delete map- 2D Map deletion done. Time cost: "<<(time_end2-time_start2)<<" s\n";

     if(marker_pub.getNumSubscribers()){
         map2D.showInital(marker_pub,robot,0);
         cout<<"change/delete map- show done\n";
     }
     //plan-not yet
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fullSys_listener");
  ros::start();
  ros::NodeHandle n;
//  demand = argv[1];
  ros::param::get("~demand",demand); //which way to create map,for test
  string pos;
  string goal;
  float resolution,slope_interval;
  ros::param::get("~pos",pos);
  ros::param::get("~goal",goal);
  ros::param::get("~resolution",resolution);
  ros::param::get("~slope_interval",slope_interval);
  robot.setPos(pos);
  robot.setGoal(goal);
  map2D.setLen(resolution);
  map2D.setInterval(slope_interval);

  marker_pub = n.advertise<visualization_msgs::MarkerArray>("initial_marker_array", 1000);
  markerArray_pub = n.advertise<visualization_msgs::MarkerArray>("traversibility_marker_array", 1000);
  markerArray_pub2 = n.advertise<visualization_msgs::MarkerArray>("tra_check_marker_array", 1000);
  change_pub = n.advertise<visualization_msgs::MarkerArray>("change_marker_array", 1000);
//  del_pub = n.advertise<visualization_msgs::MarkerArray>("del_marker_array", 1000);
  marker_pub_bo = n.advertise<visualization_msgs::MarkerArray>("bottom_marker_array", 1000);
  route_pub= n.advertise<visualization_msgs::MarkerArray>("route_marker_array", 1000);

  ros::Subscriber sub = n.subscribe("publisher/cloud_fullSys", 1000, chatterCallback); //initial
//  ros::Subscriber sub_change = n.subscribe("publisher/cloud_change", 1000,changeCallback);//change-add
//  ros::Subscriber sub_change = n.subscribe("/kitti/velo/pointcloud", 100,changeCallback);
//  ros::Subscriber sub_del = n.subscribe("publisher/cloud_del", 1000, delCallback);//change-delete
  ros::spin();
  ros::shutdown();
  return 0;
}

