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
    cloud.width = 500;
    cloud.height = 80;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);
    cout<<"start\n";
    int i=0;

        for(float x=0;x<=10;x+=0.25){
//            cout<<"in\n";
            for(float y=0;y<=x*0.56;y+=0.25){
                for(float z = 0;z<=3;z+=0.05){
                cloud.points[i].x = x;
                cloud.points[i].z = y;
                cloud.points[i].y = z;
                i++;
                }
            }
        }

        for(float x=0;x<=10;x+=0.25){
                for(float z = 0;z<=3;z+=0.05){
                cloud.points[i].x = x;
                cloud.points[i].z = x*0.56;
                cloud.points[i].y = z;
                i++;
            }
        }

        for(float x=0;x<=10;x+=0.25){
                for(float z = 0;z<=3;z+=0.05){
                cloud.points[i].x = x;
                cloud.points[i].z = x*0.56-0.01;
                cloud.points[i].y = z;
                i++;
            }
        }

        for(float x=0;x<=10;x+=0.25){
                for(float z = 0;z<=3;z+=0.05){
                cloud.points[i].x = x+0.01;
                cloud.points[i].z = x*0.56;
                cloud.points[i].y = z;
                i++;
            }
        }

        for(float x=0;x<=10;x+=0.25){
                for(float z = 0;z<=3;z+=0.05){
                cloud.points[i].x = x-0.01;
                cloud.points[i].z = x*0.56;
                cloud.points[i].y = z;
                i++;
            }
        }

//        for(float x=0;x<=10;x+=0.5){
//            for(float y=0;y<=x*0.56;y+=0.5){
//                for(float z = 0;z<=3;z+=0.5){
//                    cloud.points[i].x = x+0.05;
//                    cloud.points[i].z = y+0.05;
//                    cloud.points[i].y = z+0.05;
//                i++;
//                }
//            }
//        }

//        for(float x=0;x<=10;x+=0.5){
//            for(float y=0;y<=x*0.56;y+=0.5){
//                for(float z = 0;z<=3;z+=0.5){
//                    cloud.points[i].x = x-0.05;
//                    cloud.points[i].z = y-0.05;
//                    cloud.points[i].y = z-0.05;
//                i++;
//                }
//            }
//        }

//        for(float x=0;x<=10;x+=0.5){
//            for(float y=0;y<=x*0.56;y+=0.5){
//                for(float z = 0;z<=3;z+=0.5){
//                    cloud.points[i].x = x;
//                    cloud.points[i].z = y-0.05;
//                    cloud.points[i].y = z;
//                i++;
//                }
//            }
//        }

//        for(float x=0;x<=10;x+=0.5){
//            for(float y=0;y<=x*0.56;y+=0.5){
//                for(float z = 0;z<=3;z+=0.5){
//                    cloud.points[i].x = x;
//                    cloud.points[i].z = y+0.05;
//                    cloud.points[i].y = z;
//                i++;
//                }
//            }
//        }

//        for(float x=0;x<=10;x+=0.5){
//            for(float y=0;y<=x*0.56;y+=0.5){
//                for(float z = 0;z<=3;z+=0.5){
//                    cloud.points[i].x = x-0.05;
//                    cloud.points[i].z = y;
//                    cloud.points[i].y = z;
//                i++;
//                }
//            }
//        }

//        for(float x=0;x<=10;x+=0.5){
//            for(float y=0;y<=x*0.56;y+=0.5){
//                for(float z = 0;z<=3;z+=0.5){
//                    cloud.points[i].x = x+0.05;
//                    cloud.points[i].z = y;
//                    cloud.points[i].y = z;
//                i++;
//                }
//            }
//        }


    cout<<cloud.points.size()<<endl;
    pcl::io::savePCDFileASCII("genePcd.pcd",cloud);
    cout<<i<<" done\n";
    return 0;
}
