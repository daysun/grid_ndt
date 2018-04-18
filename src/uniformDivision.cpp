#include<2Dmap.h>
#include <cstddef>
#include <vector>
#include "Stopwatch_or.h"
#include <pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/filter.h>
#include <pcl/io/io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
using namespace Eigen;
using namespace std;
typedef multimap<int,OcNode *>  MAP_INT_MORTON_MULTI;
typedef multimap<int,OcNode *>::iterator iterIntNode;


//get data from pcd file
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
  return true;
}

//get current bounding box
 void getBondingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, Vec3 & min, Vec3 & max, std::list<Vec3> & lPoints){
     for (int i=0;i<cloud->points.size();i++)
     {
         Vec3 temp(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
         lPoints.push_back(temp);
         float maxTemp = temp.maxComponent();
         float minTemp = temp.minComponent();
         if(maxTemp > max.x)
             max.x = max.y = max.z = maxTemp;
         if(minTemp < min.x)
             min.x = min.y = min.z = minTemp;
      }
 }

 void Vec3ToPCL(list<Vec3> & points, pcl::PointCloud<pcl::PointXYZ> & cloud_a){
     cloud_a.points.resize(points.size());
     list<Vec3>::iterator iter = points.begin();
     for(unsigned int i=0; i<cloud_a.points.size(); ++i)
     {
         cloud_a.points[i].x = (*iter).x;
         cloud_a.points[i].y = (*iter).y;
         cloud_a.points[i].z = (*iter).z;
         iter++;
     }
     points.clear();
 }

 bool create2DMap(TwoDmap & map2D,MAP_INT_MORTON_MULTI map_xy,MAP_INT_MORTON_MULTI map_z){
     //get all the mortons-new cell, map.push_back
     list<int>::iterator itor = map2D.morton_list.begin();
         while(itor!=map2D.morton_list.end())
         {
             Cell * cell = new Cell(*itor);
             map2D.cell_list.push_back(cell);
             //for each morton
             //---find the nodes, count the u,c, drop the points inside
             //---determine which nodes has to be stored in the map
             if(map_xy.count(*itor) == 0){
                 cout<<"wrong\n";
                 return false;
             }else{
                 iterIntNode it = map_xy.find(*itor);
                 while(it != map_xy.end()){
                     //turn points into pcl format
                     pcl::PointCloud<pcl::PointXYZ> cloud_a;
                     if((it->second)->lPoints.size() >= 3){
                         Vec3ToPCL((it->second)->lPoints,cloud_a);
                         //compute mean and covariance_matrix, then drop the pcl
                         Eigen::Matrix3f covariance_matrix; //C
                         Eigen::Vector4f xyz_centroid; //mean
                         pcl::compute3DCentroid(cloud_a,xyz_centroid);
                         pcl::computeCovarianceMatrix(cloud_a,xyz_centroid,covariance_matrix);
                         (it->second)->xyz_centroid = xyz_centroid;
                         (it->second)->covariance_matrix = covariance_matrix;
                         // if this node's up-down neighbors are free
                         //--- store in the slope  (count roughness and Normal vector, (new slope, cell.push_back)
                         // if not free, ignore them
                         if((it->second)->isSlope(map_xy)){
                             Slope * slope = new Slope();
                             cell->slope_list.push_back(slope);
                             slope->mean.x = xyz_centroid(0);
                             slope->mean.y = xyz_centroid(1);
                             slope->mean.z = xyz_centroid(2);
                             (it->second)->countRoughNormal(slope->rough,slope->normal);
 //                             cout<<slope->rough<<","<<slope->normal<<"\n------------------"<<endl;
                         }
                     }
                     it++;
                 }
             }
             itor++;
         }
     cout<<"create 2D map done.\n";
     return true;
 }


int main(int argc, char **argv) {
    MAP_INT_MORTON_MULTI map_xy,map_z;
    float gridLen = 0.1; //related with the robot's radius-changeable
    TwoDmap map2D(gridLen);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


    ///read the point from the file
   std::string cloud_path("src/oc_ndt/data/fullSys.pcd"); //test-29 sample-4.5 table-2.8 sys-18
   OcNode  * wholeNode = new  OcNode();
   Vec3 min = Vec3(FLT_MAX,FLT_MAX,FLT_MAX),
          max = Vec3(-FLT_MAX,-FLT_MAX,-FLT_MAX);

   if (!loadCloud(cloud_path,cloud))
     return -1;

   getBondingBox(cloud, min,max,wholeNode->lPoints); //get bounding box and initiate rootNode
   float len = map2D.getGridLen();
   cout<<max.x-min.x<<endl;
  // int n = (int)ceil(float((max.x-min.x)/len));   //the whole nodes are divided into n*n*n
 //  float realLen = (max.x-min.x)/n;

   ///Discrete those points into n*n*n voxels
   while(wholeNode->lPoints.size() !=0){
       Vec3 temp = wholeNode->lPoints.front();
       wholeNode->lPoints.pop_front();
       int nx = (int)ceil(float((temp.x-min.x)/len)); // belong to which line
       int ny = (int)ceil(float((temp.y-min.y)/len)); //belong to which column
       int nz = (int)ceil(float((temp.z-min.z)/len)); //belong to which height
       int morton = countMorton(nx,ny); //count morton code-xy
       if(map_xy.count(morton) == 0){// not find
           //add new node
           OcNode * node = new OcNode();
           node->lPoints.push_back(temp);
           node->morton = morton;
           node->z = nz;
           map_xy.insert(MAP_INT_MORTON_MULTI::value_type(morton,node));
           map_z.insert(MAP_INT_MORTON_MULTI::value_type(nz,node));
           map2D.morton_list.push_back(morton);
       }else{ //find
           iterIntNode beg = map_xy.lower_bound(morton);
            iterIntNode end = map_xy.upper_bound(morton);
            while(beg != end){
                if((beg->second)->z == nz){
                    (beg->second)->lPoints.push_back(temp);
                    break;
                }
                ++beg;
            }
       }
   }
   std::cout<<"division done.\n";

   ///create 2D map from those discreted voxels
   create2DMap(map2D,map_xy,map_z);
   map2D.showSlope();
   return 0;
}
