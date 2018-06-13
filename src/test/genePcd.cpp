#include"omp.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
using namespace std;

    #define PI 3.141592653
//turn data+pose to pcd
//and the data type should be selected


int main(int argc,char *argv[])
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 600;
    cloud.height = 600;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);
    int i=0;

    //a,b
    for(float x=1;x<=5.025;x+=0.025){
    for(float y = 1;y<=5;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = x*0.5+0.5;
                i++;
            }
        }
        for(float x=11-0.025;x<=15;x+=0.025){
            for(float y = 1;y<=5;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = x*(-0.5)+8.5;
                i++;
                }
        }
    //c,d
        for(float x=5;x<11;x+=0.025){
                for(float y = 1;y<5;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = 3;
                i++;
                }
        }
//        for(float x=0;x<=16;x+=0.025){
//                for(float y = 0;y<=6;y+=0.025){
//                cloud.points[i].x = x;
//                cloud.points[i].y = y;
//                cloud.points[i].z = 1;
//                i++;
//                }
//        }
        //change 1
        for(float x=0;x<=5;x+=0.025){
                for(float y = 0;y<=6;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = 1;
                i++;
                }
        }
        for(float x=6;x<=10;x+=0.025){
                for(float y = 0;y<=6;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = 1;
                i++;
                }
        }
        for(float x=11;x<=16;x+=0.025){
                for(float y = 0;y<=6;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = 1;
                i++;
                }
        }
        //change 2
        for(float x=5;x<=6;x+=0.025){
                for(float y = 0;y<=1;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = 1;
                i++;
                }
        }
        for(float x=5;x<=6;x+=0.025){
                for(float y = 5;y<=6;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = 1;
                i++;
                }
        }
        for(float x=10;x<=11;x+=0.025){
                for(float y = 0;y<=1;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = 1;
                i++;
                }
        }
        for(float x=10;x<=11;x+=0.025){
                for(float y = 5;y<=6;y+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = y;
                cloud.points[i].z = 1;
                i++;
                }
        }

        //e,k,f,l
        for(float x=5;x<=6;x+=0.025){
                for(float z = 1;z<=3;z+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = 1;
                cloud.points[i].z = z;
                i++;
                }
        }
        for(float x=10;x<=11;x+=0.025){
                for(float z = 1;z<=3;z+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = 1;
                cloud.points[i].z = z;
                i++;
                }
        }
        for(float x=5;x<=6;x+=0.025){
                for(float z = 1;z<=3;z+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = 5;
                cloud.points[i].z = z;
                i++;
                }
        }
        for(float x=10;x<=11;x+=0.025){
                for(float z = 1;z<=3;z+=0.025){
                cloud.points[i].x = x;
                cloud.points[i].y = 5;
                cloud.points[i].z = z;
                i++;
                }
        }

        //g,h,i,j
        for(float y=1;y<=5;y+=0.025){
                for(float z = 1;z<=3;z+=0.025){
                cloud.points[i].x = 5;
                cloud.points[i].y = y;
                cloud.points[i].z = z;
                i++;
                }
        }
        for(float y=1;y<=5;y+=0.025){
                for(float z = 1;z<=3;z+=0.025){
                cloud.points[i].x = 6;
                cloud.points[i].y = y;
                cloud.points[i].z = z;
                i++;
                }
        }
        for(float y=1;y<=5;y+=0.025){
                for(float z = 1;z<=3;z+=0.025){
                cloud.points[i].x = 10;
                cloud.points[i].y = y;
                cloud.points[i].z = z;
                i++;
                }
        }
        for(float y=1;y<=5;y+=0.025){
                for(float z = 1;z<=3;z+=0.025){
                cloud.points[i].x = 11;
                cloud.points[i].y = y;
                cloud.points[i].z = z;
                i++;
                }
        }


    cout<<cloud.points.size()<<endl;
    pcl::io::savePCDFileASCII("bridge_ground.pcd",cloud);
    cout<<i<<" done\n";
    return 0;
}
