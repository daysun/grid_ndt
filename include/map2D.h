#ifndef MAP2D_H
#define MAP2D_H

#include<iostream>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include<list>
#include "robot.h"
#include<float.h>
#include<map>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace Eigen;
using namespace std;
using namespace octomath;

#define MINPOINTSIZE 3

namespace daysun{
struct CmpByKeyUD {
  bool operator()(const int& k1, const int& k2) {
      return k1<k2;
  }
};

//new one - ABCD UD
struct OcNode
{
//    Vec3 min,max;
    std::list<pcl::PointXYZ> lPoints;//to be add
    ///turn lPoints into point_cloud_ptr
    pcl::PointCloud<pcl::PointXYZ> test_cloud;
    std::list<pcl::PointXYZ> lDelPoints;//to be delete
    Eigen::Matrix3f covariance_matrix; //C
    Eigen::Vector3f xyz_centroid; //mean
    int N; //number of points which has been droped(mean having been calculated )
    string morton;
    bool _isSlope;
    int z;
    OcNode //Constructor
    (int z =0,string morton = "")
        : z(z){
        covariance_matrix = Eigen::Matrix3f::Zero(3,3);
        xyz_centroid = Eigen::Vector3f::Zero();
        N =0;
    }

    bool isEmpty(){
        if((N < MINPOINTSIZE) /*&& (covariance_matrix == Eigen::Matrix3f::Zero(3,3)) && (xyz_centroid == Eigen::Vector3f::Zero())*/)
            return true;// emtpty
        else return false; //full
    }

    //now for change-no use
    bool isSlope(multimap<string,OcNode *> & map_xy,bool & up, bool & down,float slope_interval){
            if(N<MINPOINTSIZE)
                return false;
            int zadd = z+1;
            int zminus = z-1;
            if(z == -1){
                zadd = 1;
            }else if(z == 1){
                zminus = -1;
            }
    //        cout<<"add1,minus1:"<<zadd<<","<<zminus<<endl;

            multimap<string,OcNode *>::iterator it = map_xy.find(morton);
            bool zup=false,zdown=false;
            while(it != map_xy.end()){
                if((it->first).compare(morton) != 0)
                    break;
                if(zup == true && zdown == true)
                    break;
                if(  (zminus == (it->second)->z) && abs(((it->second)->xyz_centroid)(2) - xyz_centroid(2))>slope_interval){
                    //when height(mean.z) difference are bigger than 0.5, those two nodes are believed to belong to difference level
                    zdown = true; //exist
//                    if(!(it->second)->isEmpty())
                        down = true;
                }/*else if((zminus == (it->second)->z) && abs(((it->second)->xyz_centroid)(2) - xyz_centroid(2))<slope_interval) {
                    _isSlope = false;
                    return false;
                }*/
                if((zadd == (it->second)->z) && abs(((it->second)->xyz_centroid)(2) - xyz_centroid(2))>slope_interval){
                    zup = true; //exist
//                    if(!(it->second)->isEmpty())
                        up = true;
                }
                it++;
            }
            if(up==true /*&& down == true*/){
                _isSlope = false;
                return false;
            }
            else{
                _isSlope = true;
                return true;}
    }

    void countRoughNormal(float & roughness,Vector3f & normalVector){
        EigenSolver<Matrix3f> es(covariance_matrix);
        Matrix3f eigenvalue  = es.pseudoEigenvalueMatrix(); //value
        Matrix3f eigenvector = es.pseudoEigenvectors();  //vector
        if(eigenvalue(0,0) < eigenvalue(1,1)){
            if(eigenvalue(0,0) < eigenvalue(2,2)){
                roughness = eigenvalue(0,0) ;
                normalVector = eigenvector.col(0);
            }else{
                roughness = eigenvalue(2,2) ;
                normalVector = eigenvector.col(2);
            }
        }else{
            if(eigenvalue(1,1) < eigenvalue(2,2)){
                roughness = eigenvalue(1,1) ;
                normalVector = eigenvector.col(1);
            }else{
                roughness = eigenvalue(2,2) ;
                normalVector = eigenvector.col(2);
            }
        }
        if(roughness == 0) //for visualization
            roughness = 0.01;
    }
};

struct Slope{
    Vector3f normal; //Normal vector of the slope
    float rough;  //roughness of the slope
     Vector3 mean;  //mean value of the slope
     float h;
     float g;
     float f;  //f = g+h
     string morton_xy;
     int morton_z;
     bool up, down; //false-empty true-full
     Slope * father;
     bool countUp(multimap<string,OcNode *> & map_xy,float slope_interval){//for comparision with 3D
//         int currentZ = strToInt( morton_z.substr(1,morton_z.length()-1)); // this slope
//         string belong = morton_z.substr(0,1);
//         string zadd = stringAndFloat(belong,(currentZ +1));
//         if(currentZ == 1){
//             belong.compare("U") == 0? belong="D":belong="U";
//             currentZ +=1;
//         }
         int zadd  = morton_z+1;
         int zminus = morton_z-1;
         if(morton_z == -1){
             zadd = 1;
         }else if(morton_z == 1){
             zminus = -1;
         }

         multimap<string,OcNode *>::iterator it = map_xy.find(morton_xy);
         bool zup=false;
         while(it != map_xy.end()){
             if((it->first).compare(morton_xy) != 0)
                 break;
             if(zup == true)
                 break;
             if((zadd == (it->second)->z) && abs(((it->second)->xyz_centroid)(2) - mean(2))>slope_interval){
                 zup = true; //exist
             }
             it++;
         }
         up = zup;
         return zup;
     }
};


class Cell{
    string morton;
public:
    map<int,Slope *,CmpByKeyUD> map_slope; //z, slope
    Cell(const string morton):morton(morton){}
    string getMorton(){return morton;}

};


class TwoDmap {
    float gridLen; //resolution
    Vector3 cloudFirst; //initial--the first node--deem as (0,0,0)
    float slope_interval;//interval of slope

    //adjust the ABCD and count out the left right forward and backward
    void countLRFB(string belongXY,int x,int y,string & leftMtn,string & rightMtn,string & forMtn,string &backMtn){
        int leftx,rightx,forwardx,backx,lefty,righty,forwardy,backy;
        string leftBe,rightBe,forBe,backBe;
        leftBe = rightBe = forBe = backBe = belongXY;
        leftx = x; rightx = x;
        forwardy = y;backy = y;
        if(belongXY.compare("A") == 0){
            forwardx = x+1;backx = x-1;
            lefty = y-1;righty = y+1;
        }
        if(belongXY.compare("B") == 0){
            forwardx = x+1;backx = x-1;
            lefty = y+1;righty = y-1;
        }
        if(belongXY.compare("C") == 0){
            forwardx = x-1;backx = x+1;
            lefty = y-1;righty = y+1;
        }
        if(belongXY.compare("D") == 0){
            forwardx = x-1;backx = x+1;
            lefty = y+1;righty = y-1;
        }
        if(x==1){
            if(belongXY.compare("A") == 0){
                backBe = "C";
                backx = 1;
            }
            if(belongXY.compare("B") == 0){
                backBe = "D";
                backx = 1;
            }
            if(belongXY.compare("C") == 0){
                forBe = "A";
                forwardx = 1;
            }
            if(belongXY.compare("D") == 0){
                forBe = "B";
                forwardx = 1;
            }
        }
        if(y == 1){
            if(belongXY.compare("A") == 0){
                leftBe = "B";
                lefty = 1;
            }
            if(belongXY.compare("C") == 0){
                leftBe = "D";
                lefty = 1;
            }
            if(belongXY.compare("B") == 0){
                rightBe = "A";
                righty = 1;
            }
            if(belongXY.compare("D") == 0){
                rightBe = "C";
                righty = 1;
            }
        }
//        cout<<"count "<<leftBe<<","<<leftx<<","<<lefty<<endl;
//        cout<<"count "<<rightBe<<","<<rightx<<","<<righty<<endl;
//        cout<<"count "<<forBe<<","<<forwardx<<","<<forwardy<<endl;
//        cout<<"count "<<backBe<<","<<backx<<","<<backy<<endl;
        leftMtn = leftBe+countMorton(leftx,lefty);
        rightMtn = rightBe+countMorton(rightx,righty);
        forMtn = forBe+countMorton(forwardx,forwardy);
        backMtn = backBe+countMorton(backx,backy);
    }

    //for common use/collisionCheck use
    void countReachable(string leftMtn, list<Slope *> & listm,RobotSphere & robot,
                        int morton_z,Vector3f normal,Vector3 mean,float comand){
        //find the corresponding cell-mortonxy
        map<string,Cell *>::iterator mit=  map_cell.find(leftMtn);
        if(mit != map_cell.end()){
            map<int,Slope *,CmpByKeyUD>::iterator sit =   (mit->second)->map_slope.begin();
            while(sit != (mit->second)->map_slope.end()){
                //judge if it's traversible -morton_z normal rough
                if(comand == 4){//for 3D path plan
                    sit->second->up = sit->second->countUp(map_xy,getInterval());
                    if(sit->second->up != true){
                        if((sit->second)->rough <= robot.getRough())
                            if(countAngle((sit->second)->normal,normal) <= robot.getAngle())
                                if(abs((sit->second)->mean(2) - mean(2) )<= robot.getReachableHeight())//use mean to compute
                                    listm.push_back(sit->second);
                    }
                }else if(comand ==2.5){
                  if(sit->second->up != true){
                      if((sit->second)->rough <= robot.getRough())
                          if(countAngle((sit->second)->normal,normal) <= robot.getAngle())
                              if(abs((sit->second)->mean(2) - mean(2) )<= robot.getReachableHeight())//use mean to compute
                                  listm.push_back(sit->second);
                  }
                }else if(comand ==3){
                    listm.push_back(sit->second);
                }
                sit++;
             }
         }
    }

    void countReachable(string leftMtn, list<Slope *> & listm,RobotSphere & robot,
                        int morton_z,Vector3f normal,Vector3 mean,list<Slope *> & checkList){
        //find the corresponding cell-mortonxy
        map<string,Cell *>::iterator mit=  map_cell.find(leftMtn);
        if(mit != map_cell.end()){
            map<int,Slope *,CmpByKeyUD>::iterator sit =   (mit->second)->map_slope.begin();
            while(sit != (mit->second)->map_slope.end()){
                //judge if it's traversible -morton_z normal rough
                  if(sit->second->up != true){
                      checkList.push_back(sit->second);
                      if((sit->second)->rough <= robot.getRough())
                          if(countAngle((sit->second)->normal,normal) <= robot.getAngle())
                              if(abs((sit->second)->mean(2) - mean(2) )<= robot.getReachableHeight())//use mean to compute
                                  listm.push_back(sit->second);
                  }
                  sit++;
             }
         }
    }

    //for comparision with 3D
    void countReachable3D(string leftMtn, list<Slope *> & listm,RobotSphere & robot,
                        int morton_z,Vector3f normal,Vector3 mean,list<Slope *> & checkList){
        //find the corresponding cell-mortonxy
        map<string,Cell *>::iterator mit=  map_cell.find(leftMtn);
        if(mit != map_cell.end()){
            map<int,Slope *,CmpByKeyUD>::iterator sit =   (mit->second)->map_slope.begin();
            while(sit != (mit->second)->map_slope.end()){
                checkList.push_back(sit->second);//3d has no up/down info
                    if((sit->second)->rough <= robot.getRough())
                        if(countAngle((sit->second)->normal,normal) <= robot.getAngle())
                            if(abs((sit->second)->mean(2) - mean(2) )<= robot.getReachableHeight())//use mean to compute
                                listm.push_back(sit->second);
                sit++;
             }
         }
    }

    bool isContainedQ(Slope * s,  list<Slope *> & Q){
        list<Slope *>::iterator it = Q.begin();
        while(it != Q.end()){
            if(s->morton_xy.compare((*it)->morton_xy )==0 && (s->morton_z == (*it)->morton_z) )
                return true;
            it++;
        }
        return false;
    }

    //true-collide, false-no collide
    bool CollisionCheck(Slope * slope,int n,RobotSphere & robot){
        float r =robot.getRobotR();//radius
        if(slope->up == true){
            cout<<"up collide\n";
            return true; //collide
        }
        //find all the surrounding neighbors
        list<Slope *> nowSlope,addSlope,allSlope;
        nowSlope.clear();addSlope.clear();
        allSlope.push_back(slope);
        nowSlope.push_back(slope);
        while(n>0){
            list<Slope *>::iterator itSlope = nowSlope.begin();
            while(itSlope != nowSlope.end()){
                list<Slope *> neiSlope = AccessibleNeighbors(*itSlope,robot,2.5);
                list<Slope *>::iterator itN = neiSlope.begin();
                while(itN != neiSlope.end()){
                    if(!isContainedQ(*itN,allSlope)){
                        addSlope.push_back(*itN);
                        allSlope.push_back(*itN);
                    }
                    itN++;
                }
                itSlope++;
            }
            n--;
            nowSlope.clear();
            nowSlope = addSlope;
            addSlope.clear();
        }

        list<Slope *>::iterator itSlope = allSlope.begin();
        while(itSlope != allSlope.end()){
            if(((*itSlope)->mean(2) < slope->mean(2)) && (*itSlope)->up == true){
                return true;//collide
            }
            if(((*itSlope)->mean(2) > slope->mean(2) ) && (((*itSlope)->mean(2) < slope->mean(2))+2*r)&&
                    ((*itSlope)->mean(2) - slope->mean(2) >robot.getReachableHeight())){
                return true;//collide
            }
            itSlope++;
        }

        string xy = slope->morton_xy;
        int z= slope->morton_z;
        map<string,Cell *>::iterator it = map_cell.find(xy);
        if(it != map_cell.end()){
                map<int,Slope *,CmpByKeyUD>::iterator ss = (it->second)->map_slope.find(z); //Ascending order
                if(ss == (it->second)->map_slope.end()) cout<<"collide wrong\n";
                ss++; //the next one
                if(ss != (it->second)->map_slope.end()){
//                    int height = mtnZToNum((ss->second)->morton_z);
//                    if(height < (mtnZToNum(z)+2*r/gridLen))
//                        return true;//collide
//                    else
//                        return false;//no collide
                    if(((ss->second)->mean(2) < slope->mean(2) + 2*r) &&
                            ((ss->second)->mean(2) - slope->mean(2) >robot.getReachableHeight()))
                        return true;//collide
                    else
                        return false;//no collide

                }
                return false;//no collide
        }
    }

    //for comparision with 3D
    bool CollisionCheck3D(Slope * slope,int n,RobotSphere & robot){
        float r =robot.getRobotR();//radius
        //if(slope->up == true){
        if(slope->countUp(map_xy,getInterval()) == true){
            cout<<"up collide\n";
            return true; //collide
        }
        //find all the surrounding neighbors
        list<Slope *> nowSlope,addSlope,allSlope;
        nowSlope.clear();addSlope.clear();
        allSlope.push_back(slope);
        nowSlope.push_back(slope);
        while(n>0){
            list<Slope *>::iterator itSlope = nowSlope.begin();
            while(itSlope != nowSlope.end()){
                list<Slope *> neiSlope = AccessibleNeighbors(*itSlope,robot,3);//3d
                list<Slope *>::iterator itN = neiSlope.begin();
                while(itN != neiSlope.end()){
                    if(!isContainedQ(*itN,allSlope)){
                        addSlope.push_back(*itN);
                        allSlope.push_back(*itN);
                    }
                    itN++;
                }
                itSlope++;
            }
            n--;
            nowSlope.clear();
            nowSlope = addSlope;
            addSlope.clear();
        }

        list<Slope *>::iterator itSlope = allSlope.begin();
        while(itSlope != allSlope.end()){
            if(((*itSlope)->mean(2) < slope->mean(2)) && (*itSlope)->countUp(map_xy,getInterval()) == true){
                return true;//collide
            }
            if(((*itSlope)->mean(2) > slope->mean(2) ) && (((*itSlope)->mean(2) < slope->mean(2))+2*r)&&
                    ((*itSlope)->mean(2) - slope->mean(2) >robot.getReachableHeight())){
                return true;//collide
            }
            itSlope++;
        }

        string xy = slope->morton_xy;
        int z= slope->morton_z;
        map<string,Cell *>::iterator it = map_cell.find(xy);
        if(it != map_cell.end()){
                map<int,Slope *,CmpByKeyUD>::iterator ss = (it->second)->map_slope.find(z); //Ascending order
                if(ss == (it->second)->map_slope.end()) cout<<"collide wrong\n";
                ss++; //the next one
                if(ss != (it->second)->map_slope.end()){
                    if(((ss->second)->mean(2) < slope->mean(2) + 2*r) &&
                            ((ss->second)->mean(2) - slope->mean(2) >robot.getReachableHeight()))
                        return true;//collide
                    else
                        return false;//no collide

                }
                return false;//no collide
        }
    }

    float countAngle(Vector3f n1, Vector3f n2){
        float res = n1.dot(n2) / (sqrt(pow(n1[0],2) + pow(n1[1],2)+pow(n1[2],2))* sqrt(pow(n2[0],2) + pow(n2[1],2)+pow(n2[2],2)));
        float an = acos(res)*180/M_PI;
        an>90? an = 180-an:an=an;
        return  an;
    }

public:
    multimap<string,OcNode *>  map_xy/*,map_z*/; //ABCD+morton, UD+height---index
//    multimap<string,OcNode *,CmpByKeyUD> map_z;
    TwoDmap(const float res):gridLen(res){}
    float getGridLen(){return gridLen;}
    void setCloudFirst(Vector3 p){
        cloudFirst = p;
    }
    void setLen(float len){
        gridLen = len;
    }
    void setInterval(float interval){
        slope_interval = interval;
    }
    float getInterval(){return slope_interval;}

    list<string> morton_list; //xy-morton-all
    list<string> changeMorton_list; //temp-change
    list<string> delMorton_list;//temp-delete
    map<string,Cell *> map_cell; //xy_morton, cell

    //find slope based on position
    ///no use
  Slope *  findSlope(Vector3 pos, string & morton_xy,int & morton_z){
//         transMortonXYZ(pos,morton_xy,morton_z);
         map<string,Cell *>::iterator it = map_cell.find(morton_xy);
         Slope * p = NULL;
         if(it != map_cell.end()){
             map<int,Slope *,CmpByKeyUD>::iterator ss = (it->second)->map_slope.find(morton_z);
             if(ss != (it->second)->map_slope.end()){
                 return ss->second;
             }
         }
    }

  //for now-- no consideration for the height of destination
  float TravelCost(Vector3 cur,Vector3 des,float goal = 0){
      float cost = sqrt(pow(cur(0)-des(0),2) + pow(cur(1)-des(1),2));
        return cost;
  }

  //for common use /collisionCheck use
  list<Slope *>  AccessibleNeighbors(Slope * slope,RobotSphere & robot,float comand){
//         cout<<"access slope: "<<slope->morton_xy<<","<<slope->morton_z<<endl;
      list<Slope *> list;
      string morton_xy = slope->morton_xy;
      int morton_z= slope->morton_z;
      Vector3f normal = slope->normal;
      Vector3 mean = slope->mean;
      int x,y;
      string belongXY = morton_xy.substr(0,1);
      int morton = strToInt( morton_xy.substr(1,morton_xy.length()-1));
      mortonToXY(x,y,morton);
      string leftMtn,rightMtn,forMtn,backMtn;
      countLRFB(belongXY,x,y,leftMtn,rightMtn,forMtn,backMtn); //adjust belong
      countReachable(leftMtn,list,robot,morton_z,normal,mean,comand);
      countReachable(rightMtn,list,robot,morton_z,normal,mean,comand);
      countReachable(forMtn,list,robot,morton_z,normal,mean,comand);
      countReachable(backMtn,list,robot,morton_z,normal,mean,comand);
      return list;
  }

    //find the reachable surrounding slopes
     list<Slope *>  AccessibleNeighbors(Slope * slope,RobotSphere & robot,list<Slope *> & checkList){
//         cout<<"access slope: "<<slope->morton_xy<<","<<slope->morton_z<<endl;
         list<Slope *> list;
         string morton_xy = slope->morton_xy;
         int morton_z= slope->morton_z;
         Vector3f normal = slope->normal;
         Vector3 mean = slope->mean;
         int x,y;
         string belongXY = morton_xy.substr(0,1);
         int morton = strToInt( morton_xy.substr(1,morton_xy.length()-1));
         mortonToXY(x,y,morton);
         string leftMtn,rightMtn,forMtn,backMtn;
         countLRFB(belongXY,x,y,leftMtn,rightMtn,forMtn,backMtn); //adjust belong
         countReachable(leftMtn,list,robot,morton_z,normal,mean,checkList);
         countReachable(rightMtn,list,robot,morton_z,normal,mean,checkList);
         countReachable(forMtn,list,robot,morton_z,normal,mean,checkList);
         countReachable(backMtn,list,robot,morton_z,normal,mean,checkList);
         return list;
     }

     //for comparision with 3D
      list<Slope *>  AccessibleNeighbors3D(Slope * slope,RobotSphere & robot,list<Slope *> & checkList){
 //         cout<<"access slope: "<<slope->morton_xy<<","<<slope->morton_z<<endl;
          list<Slope *> list;
          string morton_xy = slope->morton_xy;
          int morton_z= slope->morton_z;
          Vector3f normal = slope->normal;
          Vector3 mean = slope->mean;
          int x,y;
          string belongXY = morton_xy.substr(0,1);
          int morton = strToInt( morton_xy.substr(1,morton_xy.length()-1));
          mortonToXY(x,y,morton);
          string leftMtn,rightMtn,forMtn,backMtn;
          countLRFB(belongXY,x,y,leftMtn,rightMtn,forMtn,backMtn); //adjust belong
          countReachable3D(leftMtn,list,robot,morton_z,normal,mean,checkList);
          countReachable3D(rightMtn,list,robot,morton_z,normal,mean,checkList);
          countReachable3D(forMtn,list,robot,morton_z,normal,mean,checkList);
          countReachable3D(backMtn,list,robot,morton_z,normal,mean,checkList);
          return list;
      }

     bool create2DMap(string demand){
//         cout<<"enter create\n";
         //get all the mortons-new cell, map.push_back
         list<string>::iterator itor = morton_list.begin();
             while(itor!=morton_list.end())
             {
                 Cell * cell = new Cell(*itor);
                 map_cell.insert(map<string,Cell*>::value_type(cell->getMorton(), cell));
                 map<string,OcNode *,CmpByKeyUD> temp_cellZ;
                 temp_cellZ.clear();
                 if(map_xy.count(*itor) == 0){
                     cout<<"wrong\n";
                     return false;
                 }else{
                     multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(*itor);
                     while(it != map_xy.end()){
                         if((it->first).compare(*itor) != 0)
                             break;
//                         if((it->second)->lPoints.size() >= MINPOINTSIZE){
                         if((it->second)->test_cloud.points.size() >= MINPOINTSIZE){
//                             pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//                             std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lPoints.begin();
//                             while(node_iter != (it->second)->lPoints.end()){
//                                  point_cloud_ptr->points.push_back (*node_iter);
//                                  node_iter++;
//                             }
                             //compute mean and covariance_matrix, then drop the pcl
                             Eigen::Matrix3f covariance_matrix; //C
                             Eigen::Vector4f xyz_centroid; //mean
                             pcl::compute3DCentroid((it->second)->test_cloud,xyz_centroid);
                             pcl::computeCovarianceMatrix((it->second)->test_cloud,xyz_centroid,covariance_matrix);
                             (it->second)->xyz_centroid <<xyz_centroid(0),xyz_centroid(1),xyz_centroid(2);
                             (it->second)->covariance_matrix = covariance_matrix;
                             (it->second)->N += (it->second)->test_cloud.points.size(); //record the num counting mean and C
//                             (it->second)->lPoints.clear();
                             (it->second)->test_cloud.points.clear();
                             bool up= false, down = false;
//                             for test
                             if(demand.compare("slope") == 0){
                                 if((it->second)->isSlope(map_xy,up,down,slope_interval) ){
                                     Slope * slope = new Slope();
                                     cell->map_slope.insert(make_pair((it->second)->z,slope));
                                     slope->morton_xy = (it->second)->morton;
                                     slope->morton_z= (it->second)->z;
                                     /*slope->up = up,*/slope->down = down;
                                     slope->h = slope->g = slope->f = FLT_MAX;
                                     slope->mean(0) = (it->second)->xyz_centroid(0);
                                     slope->mean(1) = (it->second)->xyz_centroid(1);
                                     slope->mean(2) = (it->second)->xyz_centroid(2);
                                     slope->father = NULL;//for path plan
                                     (it->second)->countRoughNormal(slope->rough,slope->normal);
                                 }
                             }else if(demand.compare("true") == 0){
                                 if( true){
                                     //3D without up/down information
//                                     (it->second)->isSlope(map_xy,up,down,slope_interval);
                                     Slope * slope = new Slope();
                                     cell->map_slope.insert(make_pair((it->second)->z,slope));
                                     slope->morton_xy = (it->second)->morton;
                                     slope->morton_z= (it->second)->z;
                                     /*slope->up = up,*/slope->down = down;//no use in 3D
                                     slope->h = slope->g = slope->f = FLT_MAX;
                                     slope->mean(0) = (it->second)->xyz_centroid(0);
                                     slope->mean(1) = (it->second)->xyz_centroid(1);
                                     slope->mean(2) = (it->second)->xyz_centroid(2);
                                     slope->father = NULL;//for path plan
                                     (it->second)->countRoughNormal(slope->rough,slope->normal);
                                 }
                             }
                         }
                         it++;
                     }// else while end
                 }//else end
                 itor++;
             }//while end
         return true;
     }

    // change-add map
    ///no consideration for now, maybe wrong
    bool change2DMap(){
        cout<<"start change 2D map\n";
        if(changeMorton_list.size() == 0)
            return false;
        list<string>::iterator itor = changeMorton_list.begin();
        while(itor != changeMorton_list.end()){
            //find if it's contained in cell_morton_list
            if(map_cell.size() == 0)
                return false;
            string changeMorton = *itor;
            map<string,Cell *>::iterator map_it= map_cell.find(changeMorton);
            if(map_it == map_cell.end()) {
                //not find 1
                //create a new cell- the same as the initial
                Cell * cell = new Cell(*itor);
                map_cell.insert(map<string,Cell*>::value_type(cell->getMorton(), cell));
                if(map_xy.count(*itor) == 0){
                    cout<<"wrong\n";
                    return false;
                }else{
                    multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(*itor);
                    while(it != map_xy.end()){
                        if((it->first).compare(*itor) != 0)
                            break;
                        if((it->second)->lPoints.size() >= MINPOINTSIZE){
                            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                            std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lPoints.begin();
                            while(node_iter != (it->second)->lPoints.end()){
                                 point_cloud_ptr->points.push_back (*node_iter);
                                 node_iter++;
                            }
                            Eigen::Matrix3f covariance_matrix; //C
                            Eigen::Vector4f xyz_centroid; //mean
                            pcl::compute3DCentroid(*point_cloud_ptr,xyz_centroid);
                            pcl::computeCovarianceMatrix(*point_cloud_ptr,xyz_centroid,covariance_matrix);
                            (it->second)->xyz_centroid << xyz_centroid(0),xyz_centroid(1),xyz_centroid(2);
                            (it->second)->covariance_matrix = covariance_matrix;
                            (it->second)->N += (it->second)->lPoints.size();
                            (it->second)->lPoints.clear();
                            bool up= false, down = false;
                            if((it->second)->isSlope(map_xy,up,down,slope_interval) ){
                                Slope * slope = new Slope();
                                cell->map_slope.insert(make_pair((it->second)->z,slope));
                                slope->morton_xy = (it->second)->morton;
                                slope->morton_z= (it->second)->z;
                                slope->up = up,slope->down = down;
                                slope->father = NULL;
                                slope->h = slope->g = slope->f = FLT_MAX;
                                (it->second)->countRoughNormal(slope->rough,slope->normal);
                            }
                        }
                        it++;
                    }
                }
            }else{
                //find 1-handle the exsit cell-update it
                Cell * cell = map_it->second; //get the exist cell
                multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(changeMorton);
                //for each node morton_xy==target
                while(it != map_xy.end()){
                    if((it->first).compare(changeMorton) != 0)
                        break;
                    if((it->second)->lPoints.size() >= MINPOINTSIZE){
                        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                        std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lPoints.begin();
                        while(node_iter != (it->second)->lPoints.end()){
                             point_cloud_ptr->points.push_back (*node_iter);
                             node_iter++;
                        }
                        if((Eigen::Matrix3f::Zero(3,3) == (it->second)->covariance_matrix )
                                && (Eigen::Vector3f::Zero()==  (it->second)->xyz_centroid)){
                            //not existed--new node
                            Eigen::Matrix3f covariance_matrix; //C
                            Eigen::Vector4f xyz_centroid; //mean
                            pcl::compute3DCentroid(*point_cloud_ptr,xyz_centroid);
                            pcl::computeCovarianceMatrix(*point_cloud_ptr,xyz_centroid,covariance_matrix);
                            (it->second)->xyz_centroid <<xyz_centroid(0),xyz_centroid(1),xyz_centroid(2);
                            (it->second)->covariance_matrix = covariance_matrix;
                            (it->second)->N += (it->second)->lPoints.size();
                            (it->second)->lPoints.clear();
                            //new Slope
                            bool up= false, down = false;
                            if((it->second)->isSlope(map_xy,up,down,slope_interval)){
                                Slope * slope = new Slope();
                                cell->map_slope.insert(make_pair((it->second)->z,slope));
                                slope->morton_xy = (it->second)->morton;
                                slope->morton_z= (it->second)->z;
                                slope->up = up,slope->down = down;
                                slope->father = NULL;
                                slope->h = slope->g = slope->f = FLT_MAX;
                                (it->second)->countRoughNormal(slope->rough,slope->normal);
                             }
                        }else{
                            bool up= false, down = false;
                            bool beforeSlope = (it->second)->isSlope(map_xy,up,down,slope_interval); //before delete isSlope
                             Eigen::Matrix3f C0 = (it->second)->covariance_matrix;
                             Eigen::Vector3f u0 = (it->second)->xyz_centroid;
                             int N =  (it->second)->N ;
                             int M =  (it->second)->lPoints.size();
                             Eigen::Matrix3f C1,covariance_matrix;
                             Eigen::Vector4f temp_u;
                             Eigen::Vector3f u1,xyz_centroid;
                             pcl::compute3DCentroid(*point_cloud_ptr,temp_u);
                             pcl::computeCovarianceMatrix(*point_cloud_ptr,temp_u,C1);
                             u1<<temp_u(0),temp_u(1),temp_u(2);
                             xyz_centroid =
                                     ( N*u0 + M*u1) / (M+N);
                             covariance_matrix =
                                     ((N-1)*C0 + (M-1)*C1 + M*N/(M+N)*((u0-u1)*((u0-u1).transpose()))) / (M+N-1);
                             (it->second)->xyz_centroid = xyz_centroid;
                             (it->second)->covariance_matrix = covariance_matrix;
                             (it->second)->N += (it->second)->lPoints.size();
                             (it->second)->lPoints.clear();
                             if(beforeSlope){
                                 map<int,Slope *,CmpByKeyUD>::iterator cellItor = cell->map_slope.find((it->second)->z);
                                 if(cellItor == cell->map_slope.end()){
                                     cout<<"cant find slope by z, error\n";
                                     return false;
                                 }else{
                                     if((it->second)->isSlope(map_xy,up,down,slope_interval)){
                                         //change the old slope's u,C
                                         (cellItor->second)->up = up,(cellItor->second)->down = down;
                                         (it->second)->countRoughNormal((cellItor->second)->rough,(cellItor->second)->normal);
                                     }else{
                                         //delete the old slope
                                         cell->map_slope.erase(cellItor++);
                                     }
                                 }
                             }else{
                                 if((it->second)->isSlope(map_xy,up,down,slope_interval)){
                                     //new Slope
                                     Slope * slope = new Slope();
                                     cell->map_slope.insert(make_pair((it->second)->z,slope));
                                     slope->morton_xy = (it->second)->morton;
                                     slope->morton_z= (it->second)->z;
                                     slope->up = up,slope->down = down;
                                     slope->father = NULL;
                                     slope->h = slope->g = slope->f = FLT_MAX;
                                     (it->second)->countRoughNormal(slope->rough,slope->normal);
                                 }
                             }
                        }
                    }
                    it++;
                }
            }//else end
             itor++;
        } //change morton list end
            cout<<"change 2D map done.\n";
            return true;
    }

    //change-delete map
    ///no consideration for now, maybe wrong
    bool del2DMap(){
        cout<<"start delete 2D map\n";
        if(delMorton_list.size() == 0)
            return false;
        list<string>::iterator itor = delMorton_list.begin();
//        int delete_num = 0;
        while(itor != delMorton_list.end()){
            //find if it's contained in cell_morton_list
            if(map_cell.size() == 0)
                return false;
            string changeMorton = *itor;
            map<string,Cell *>::iterator map_it= map_cell.find(changeMorton);
            if(map_it != map_cell.end()) {
//                cout<<"changemorton \n"<<changeMorton<<endl;
                //handle the exsit cell-update node and slope
                Cell * cell = map_it->second; //get the exist cell
                multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(changeMorton);
                //for each node morton_xy==target
                while(it != map_xy.end()){
//                    cout<<"map_xy it "<<it->first<<endl;
                    if((it->first).compare(changeMorton) != 0){
                        break;
                    }
                    if((it->second)->lDelPoints.size() >= MINPOINTSIZE){
                        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                        std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lDelPoints.begin();
                        while(node_iter != (it->second)->lDelPoints.end()){
                             point_cloud_ptr->points.push_back (*node_iter);
                             node_iter++;
                        }
                        bool up= false, down = false;
                        bool beforeSlope = (it->second)->isSlope(map_xy,up,down,slope_interval); //before delete isSlope
                        Eigen::Matrix3f C0 = (it->second)->covariance_matrix;
                        Eigen::Vector3f u0 = (it->second)->xyz_centroid;
                        int N =  (it->second)->N ;
                        int M =  -(it->second)->lDelPoints.size(); ///M = -M, delete
                        if(N+M <= 0){
                            //delete node and slope
                            if(beforeSlope){
                                map<int,Slope *,CmpByKeyUD>::iterator cellItor = cell->map_slope.find((it->second)->z);
                                if(cellItor != cell->map_slope.end()){
                                    cell->map_slope.erase(cellItor);//delete slope
//                                    delete_num++;
                                }
                            }
                            //delete node
                            map_xy.erase(it++);//delete node
                            continue;
                        }else{
                            Eigen::Matrix3f C1,covariance_matrix;
                            Eigen::Vector4f temp_u;
                            Eigen::Vector3f u1,xyz_centroid;
                            pcl::compute3DCentroid(*point_cloud_ptr,temp_u);
                            pcl::computeCovarianceMatrix(*point_cloud_ptr,temp_u,C1);
                            u1<<temp_u(0),temp_u(1),temp_u(2);
                            xyz_centroid =
                                    ( N*u0 + M*u1) / (N+M);
                            covariance_matrix =
                                    ((N-1)*C0 + (M-1)*C1 + M*N/(M+N)*((u0-u1)*((u0-u1).transpose()))) / (M+N-1);
                            (it->second)->xyz_centroid = xyz_centroid;
                            (it->second)->covariance_matrix = covariance_matrix;
                            (it->second)->N -= (it->second)->lDelPoints.size();
                            (it->second)->lDelPoints.clear();

                            //slope
                            //--based on the before slope + after slope
                            if(beforeSlope){
                                map<int,Slope *,CmpByKeyUD>::iterator cellItor = cell->map_slope.find((it->second)->z);
                                if(cellItor != cell->map_slope.end()){
                                    if((it->second)->isSlope(map_xy,up,down,slope_interval)){
                                        //change the old slope's u,C
                                        (cellItor->second)->up = up,(cellItor->second)->down = down;
                                        (it->second)->countRoughNormal((cellItor->second)->rough,(cellItor->second)->normal);
                                    }else{
                                        //delete the old slope
                                        cell->map_slope.erase(cellItor);
//                                        delete_num++;
                                    }
                                }
                            }
                        }
                    }
                    it++;
                }
            }//if end
             itor++;
        } //change morton list end
//        cout<<"delete slope num "<<delete_num<<endl;
            return true;
    }

    //for visualization
    void countPositionXYZ(float & x,float &y,float &z,string s_xy,int s_z){
        string belongxy = s_xy.substr(0,1);
//        string belongz = s_z.substr(0,1);
         int mortonxy = strToInt( s_xy.substr(1,s_xy.length()-1));
//         int mortonz = strToInt( s_z.substr(1,s_z.length()-1));
         int a,b;
         mortonToXY(a,b,mortonxy);
         if(belongxy.compare("A") == 0){
             x = (a-0.5)*gridLen + cloudFirst(0);
             y = (b-0.5)*gridLen + cloudFirst(1);
         }
         if(belongxy.compare("B") == 0){
             x = (a-0.5)*gridLen + cloudFirst(0);
             y = cloudFirst(1) - (b-0.5)*gridLen ;
         }
         if(belongxy.compare("C") == 0){
             x = cloudFirst(0)- (a-0.5)*gridLen ;
             y = (b-0.5)*gridLen + cloudFirst(1);
         }
         if(belongxy.compare("D") == 0){
             x = cloudFirst(0)- (a-0.5)*gridLen ;
             y = cloudFirst(1) - (b-0.5)*gridLen ;
         }
         if(s_z >0){
             z = cloudFirst(2) + (abs(s_z)-0.5)*gridLen;
         }
         else if(s_z <0){
             z = cloudFirst(2) - (abs(s_z)-0.5)*gridLen;
         }
    }

    //tranform position into morton_xy and morton_z
    void transMortonXYZ(Vector3 position, string & morton_xy, int & morton_z){
        string xy_belong;
        if(position(0) > cloudFirst(0)){
            if(position(1) >cloudFirst(1))
                xy_belong = "A";
            else
                xy_belong = "B";
        }else{
            if(position(1)>cloudFirst(1))
                xy_belong = "C";
            else
                xy_belong = "D";
        }
        if(position(2) > cloudFirst(2)) morton_z = 1; //up
        else morton_z = -1; //down
        int nx = (int)ceil(float(abs(position(0)-cloudFirst(0))/gridLen));
        int ny = (int)ceil(float(abs(position(1)-cloudFirst(1))/gridLen));
        int nz = (int)ceil(float(abs(position(2)-cloudFirst(2))/gridLen));
        nx == 0? nx =1:nx=nx;
        ny == 0? ny =1:ny=ny;
        nz == 0? nz =1:nz=nz;
        string morton = countMorton(nx,ny);
        morton_xy = xy_belong+ morton;
        morton_z =morton_z * nz;
//        cout<<"morton_z "<<morton_z<<endl;
//        cout<<"abcd "<<t2-t1<<",nxyz "<<t3-t2<<",morton "<<t4-t3<<",str&float"<<t5-t4<<endl;
    }

    //for visualiation
    //using mean
    void showSlopeList(ros::Publisher marker_pub,list<Slope *> & closed,int color =0){
        ros::Rate r(50);
        uint32_t shape = visualization_msgs::Marker::CUBE; //SPHERE ARROW CYLINDER
        visualization_msgs::MarkerArray mArray;
        int i = 0;
//        cout<<closed.size()<<endl;
        list<Slope *>::iterator it = closed.begin();
        while(it != closed.end()){
            Vector3f normal = (*it)->normal;
//            Vec3 mean = (*it)->mean;
            float rough = (*it)->rough;
            float x,y,z;
            string s_xy = (*it)->morton_xy;
            int s_z = (*it)->morton_z;
            countPositionXYZ(x,y,z,s_xy,s_z);
                    visualization_msgs::Marker m_s;
                    m_s.ns  = "traversibility";
                    m_s.header.frame_id = "/my_frame";
                    m_s.header.stamp = ros::Time::now();
                    m_s.id = i;
                    m_s.type = shape;
                    m_s.action = visualization_msgs::Marker::ADD;
                    m_s.pose.position.x = (*it)->mean(0);
                    m_s.pose.position.y = (*it)->mean(1);
//                    m_s.pose.position.z = z;
                    m_s.pose.position.z = (*it)->mean(2);
                    m_s.pose.orientation.x = normal(0);
                    m_s.pose.orientation.y = normal(1);
                    m_s.pose.orientation.z = normal(2);
                    m_s.pose.orientation.w = 1.0;
                    m_s.scale.x = gridLen;
                    m_s.scale.y = gridLen;
                    m_s.scale.z = 2.5* rough;
                    if(color == 0){ //traversibility
                        m_s.color.a = 1.0;
                        m_s.color.b = 1.0;
                        m_s.color.r = 0.5;
                        m_s.scale.z = gridLen/2;
                    }
                    else if(color == 1){ //not traversible--check
                        m_s.color.a = 1.0;
                        m_s.color.b = 1;
                        m_s.color.r = 1;
                        m_s.color.g = 0.5;
                        m_s.scale.z = gridLen/2;
                    }
                    else if(color == 2){ //for bottom_cell
                        m_s.color.a = 1.0;
                        m_s.color.r = 0.5;
                        m_s.color.g = 0.2;
                    }else if(color == 4){ //for route
                        m_s.color.a = 1.0;
                        m_s.color.r = 1;
                        m_s.color.g = 1;
                        m_s.scale.x = gridLen;
                        m_s.scale.z = gridLen;
                    }
                    m_s.lifetime = ros::Duration();
                    mArray.markers.push_back(m_s);
            it++;i++;
            }
        if(ros::ok()){
            if (marker_pub.getNumSubscribers() == 1){
                marker_pub.publish(mArray);
                ros::spinOnce();
                   r.sleep();
            }
        }
    }

    //for visualization-initial
    //use mean
    void showInital(ros::Publisher marker_pub,RobotSphere & robot,int color =0){ //0-,1-change
        float radius=gridLen;
        ros::Rate r(50);
        int node_num = 0,slope_num=0;
        uint32_t shape = visualization_msgs::Marker::CUBE; //SPHERE ARROW CYLINDER
         visualization_msgs::MarkerArray mArray;
        int i = 2;
        if (ros::ok()){
            //1-for start and goal
            {
                list<Vector3> lv ;
                lv.push_back(robot.getGoal());
                lv.push_back(robot.getPosition());
                list<Vector3>::iterator ilv = lv.begin();
                int j = 0;
                while(ilv != lv.end()){
                    string m_xy;
                    int m_z;
                    float x,y,z;
                    transMortonXYZ(*ilv,m_xy,m_z);
                    countPositionXYZ(x,y,z,m_xy,m_z);
                    visualization_msgs::Marker m_s;
                    m_s.ns  = "basic_shapes";
                    m_s.header.frame_id = "/my_frame";
                    m_s.header.stamp = ros::Time::now();
                    m_s.id = j;
                    m_s.type = shape;
                    m_s.action = visualization_msgs::Marker::ADD;
                    m_s.pose.position.x = (*ilv)(0);
                    m_s.pose.position.y = (*ilv)(1);
//                    m_s.pose.position.z = z;
                    m_s.pose.position.z = (*ilv)(2);
                    m_s.scale.x = radius; //the same as radius
                    m_s.scale.y = radius;
                    m_s.scale.z = radius;
                    m_s.pose.orientation.x = 0;
                    m_s.pose.orientation.y = 0;
                    m_s.pose.orientation.z = 0;
                    m_s.pose.orientation.w = 1.0;
                    m_s.color.a = 0.0;
                    m_s.color.r = 0.5;
                    m_s.color.g = 0.5;
                    m_s.lifetime = ros::Duration();
                    mArray.markers.push_back(m_s);
                    ilv++;j++;
                }
            }
            //2-for every cell
            if(map_cell.size() != 0){
                map<string,Cell*>::iterator cell_iter= map_cell.begin();
                while(cell_iter != map_cell.end()){
                     Cell * cell = cell_iter->second;                                  
                     ///1-for comparision
                     /// for each node
                     /*
                     multimap<string,OcNode *>::iterator nodeIter = map_xy.find(cell->getMorton());
                     while(nodeIter != map_xy.end()){
                         if((nodeIter->second)->morton.compare(cell->getMorton()) != 0)
                             break;
                         i++;node_num++;
                        //add marker
                         if((nodeIter->second)->_isSlope != true){
                        visualization_msgs::Marker marker;
                        marker.ns = "basic_shapes";
                        marker.header.frame_id = "/my_frame";
                        marker.header.stamp = ros::Time::now();
                        marker.id = i; //same namespace and id will overwrite the old one
                        marker.type = shape;
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.position.x = (nodeIter->second)->xyz_centroid(0);
                        marker.pose.position.y = (nodeIter->second)->xyz_centroid(1);
                         marker.pose.position.z = (nodeIter->second)->xyz_centroid(2);
                        marker.pose.orientation.x = 0;
                        marker.pose.orientation.y = 0;
                        marker.pose.orientation.z = 1;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = radius-0.01; //the same as radius
                        marker.scale.y = radius-0.01;
                        marker.scale.z = 0.01;
                        marker.color.a = 0.9;
                        marker.color.r = 0.8;
                        marker.color.g =0.8;
                        marker.color.b =0.8;
                        marker.lifetime = ros::Duration();
                        mArray.markers.push_back(marker);
                        }
                         nodeIter++;
                     }*/

                     ///2-for every slope
                      map<int,Slope *,CmpByKeyUD>::iterator slItor = cell->map_slope.begin();
                      while(slItor != cell->map_slope.end()){
                          i++;slope_num++;
                         Vector3f normal = (slItor->second)->normal;
 //                            float rough = (slItor->second)->rough;
                         string s_xy = (slItor->second)->morton_xy;
                         int s_z = (slItor->second)->morton_z;
                         float x,y,z;
                         countPositionXYZ(x,y,z,s_xy,s_z);
                         //add marker
                         visualization_msgs::Marker marker;
                         marker.ns = "basic_shapes";
                         marker.header.frame_id = "/my_frame";
                         marker.header.stamp = ros::Time::now();
                         marker.id = i; //same namespace and id will overwrite the old one
                         marker.type = shape;
                         marker.action = visualization_msgs::Marker::ADD;
                         ///changed
                         marker.pose.position.x = (slItor->second)->mean(0);
                         marker.pose.position.y = (slItor->second)->mean(1);
                          marker.pose.position.z = (slItor->second)->mean(2);
//                          marker.pose.position.x = x;
//                          marker.pose.position.y = y;
//                           marker.pose.position.z = (slItor->second)->mean(2);

                          marker.pose.orientation.x = normal(0);
                          marker.pose.orientation.y = normal(1);
                          marker.pose.orientation.z = normal(2);
                          marker.pose.orientation.w = 1.0;
//                         if(countAngle(Vector3f(0,0,1),normal)> 20){
//                             marker.pose.orientation.x = 0;
//                             marker.pose.orientation.y = 2.75;
//                             marker.pose.orientation.z = 0;
//                         }

                         marker.scale.x = radius-0.01; //the same as radius
                         marker.scale.y = radius-0.01;
                         marker.scale.z = 0.03/*rough*/;
                         marker.color.a = 0.9;
                         marker.color.r = 1;
                         if(color == 1){
                             //for testing change
                             marker.color.g =1;
                             marker.color.b =1;
                         }
                         marker.lifetime = ros::Duration();
                         mArray.markers.push_back(marker);
                         slItor++;
                      }

                    cell_iter++;
                }
            }
            cout<<"node "<<node_num<<", slope "<<slope_num<<endl;
            if (marker_pub.getNumSubscribers() == 1){
                marker_pub.publish(mArray);
                ros::spinOnce();
                   r.sleep();
            }
        }
    }

    //for visualization-bottom grid
    void showBottom(ros::Publisher marker_pub){
        ros::Rate r(50);
        uint32_t shape = visualization_msgs::Marker::CUBE; //SPHERE ARROW CYLINDER
        visualization_msgs::MarkerArray mArray;
        int i =0;
        map<string,Cell *>::iterator it = map_cell.begin();
        while(it != map_cell.end()){
            Cell * cell = it->second;
            Vector3f normal ;
            normal<<0,0,1;
            float rough = 0.1;
            float x,y,z;
            string s_xy = cell->getMorton();
            int s_z = -3;
            countPositionXYZ(x,y,z,s_xy,s_z);
                    visualization_msgs::Marker m_s;
                    m_s.ns  = "bottom";
                    m_s.header.frame_id = "/my_frame";
                    m_s.header.stamp = ros::Time::now();
                    m_s.id = i;
                    m_s.type = shape;
                    m_s.action = visualization_msgs::Marker::ADD;
                    m_s.pose.position.x = x;
                    m_s.pose.position.y = y;
                    m_s.pose.position.z = z;
                    m_s.pose.orientation.x = normal(0);
                    m_s.pose.orientation.y = normal(1);
                    m_s.pose.orientation.z = normal(2);
                    m_s.pose.orientation.w = 1.0;
                    m_s.scale.x = gridLen;
                    m_s.scale.y = gridLen;
                    m_s.scale.z = 0.01/*1.5* rough*/; //not using the rough, for visualization
                        m_s.color.a = 1.0;
                        m_s.color.r = 0.5;
                        m_s.color.b = 0.5;
                    m_s.lifetime = ros::Duration();
                    mArray.markers.push_back(m_s);
            it++; i++;
        }
        if(ros::ok()){
            if (marker_pub.getNumSubscribers() == 1){
                marker_pub.publish(mArray);
                ros::spinOnce();
                   r.sleep();
            }
        }
    }

    //compute cost map
    void computeCost(Vector3 goal,RobotSphere & robot,ros::Publisher marker_pub,ros::Publisher marker_pub2,string demand){
        double time_start2 = stopwatch();
         list<Slope *> Q; //list of cell to be computed
         list<Slope *> closed;//no checking again
         list<Slope *> traversability; //can travel
         list<Slope *> checkList;
         string morton_xy;
         int morton_z;
         transMortonXYZ(goal,morton_xy,morton_z);
         map<string,Cell *>::iterator it = map_cell.find(morton_xy);
         if(it != map_cell.end()){
             map<int,Slope *,CmpByKeyUD>::iterator ss = (it->second)->map_slope.find(morton_z);
             if(ss != (it->second)->map_slope.end()){
                 ss->second->h = 0; //goal.h = 0
                 Q.push_back( ss->second);
//                 cout<<"find start\n";
             }else{
                 cout<<"Goal position wrong: cant find goal slope.\n";
                 return ;
             }
         }

         int n = (ceil(2*robot.getRobotR()/gridLen)-1)/2; //ceil-compute more
//         cout<<"n "<<n<<endl;

         //compute the surrounding morton code
         if(demand.compare("slope") == 0){
             while(Q.size() != 0){
                 if(!CollisionCheck(Q.front(),n,robot )){
                     //no collision
                     list<Slope *> neiSlope = AccessibleNeighbors(Q.front(),robot,checkList);
                     list<Slope *>::iterator itN = neiSlope.begin();
                     while(itN != neiSlope.end()){
//                         if((*itN)->up == true){
//                             (*itN)->h = FLT_MAX;
//                             closed.push_back(*itN);
//                         }else{
                             Vector3 q,itn;
                             ///use the morton_xy and morton_z
                             {
    //                         string s_xy = Q.front()->morton_xy;
    //                         string s_z = Q.front()->morton_z;
    //                         countPositionXYZ(q.x,q.y,q.z,s_xy,s_z);
    //                         string n_xy = (*itN)->morton_xy;
    //                         string n_z = (*itN)->morton_z;
    //                         countPositionXYZ(itn.x,itn.y,itn.z,n_xy,n_z);
                             }
                             ///use the mean xyz
                             q = Q.front()->mean;
                             itn = (*itN)->mean;

                             if((*itN)->h > Q.front()->h + TravelCost(q,itn,goal(2))){
                                 (*itN)->h = Q.front()->h + TravelCost(q,itn,goal(2));
                                 if(!isContainedQ(*itN,Q) && !isContainedQ(*itN,closed) && !isContainedQ(*itN,traversability)){
                                     Q.push_back(*itN);
                                 }
                             }
//                         }
                         itN++;
                     }
                     traversability.push_back(Q.front());
                 }else{
                     Q.front()->h = FLT_MAX; //collide
                     closed.push_back(Q.front());
                 }
                 Q.pop_front();
             }
         }else if(demand.compare("true") == 0){//for comaprision with 3D
             while(Q.size() != 0){
                 if(!CollisionCheck3D(Q.front(),n,robot )){
                     //no collision
                     list<Slope *> neiSlope = AccessibleNeighbors3D(Q.front(),robot,checkList);
                     list<Slope *>::iterator itN = neiSlope.begin();
                     while(itN != neiSlope.end()){
//                         if((*itN)->up == true){ ///----------------change
//                         if((*itN)->countUp(map_xy,getInterval()) == true){
//                             (*itN)->h = FLT_MAX;
//                             closed.push_back(*itN);
//                         }else{
                             Vector3 q,itn;
                             q = Q.front()->mean;
                             itn = (*itN)->mean;

                             if((*itN)->h > Q.front()->h + TravelCost(q,itn,goal(2))){
                                 (*itN)->h = Q.front()->h + TravelCost(q,itn,goal(2));
                                 if(!isContainedQ(*itN,Q) && !isContainedQ(*itN,closed) && !isContainedQ(*itN,traversability)){
                                     Q.push_back(*itN);
                                 }
                             }
//                         }
                         itN++;
                     }
                     traversability.push_back(Q.front());
                 }else{
                     Q.front()->h = FLT_MAX; //collide
                     closed.push_back(Q.front());
                 }
                 Q.pop_front();
             }
         }
         double time_end2 = stopwatch();
         cout<<"traversability time: "<<(time_end2-time_start2)<<" s\n";
         cout<<"traversability slopes "<<traversability.size()<<endl;
         cout<<"check slopes "<<checkList.size()<<endl;
         if(marker_pub.getNumSubscribers() ){
             showSlopeList(marker_pub,traversability,0);
             cout<<"costmap show done\n";
         }
         if(marker_pub2.getNumSubscribers() ){
             showSlopeList(marker_pub2,checkList,1);
             cout<<"check slope/node show done\n";
         }
    }

};

}
#endif // MAP2D_H
