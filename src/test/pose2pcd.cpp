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

//turn data+pose to pcd
//and the data type should be selected

float strToFloat(string s){
    float num;
       stringstream ss(s);
       ss >> num;
       return num;
}

void SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

Eigen::Quaterniond euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

void readTxt(string file,pcl::PointCloud<pcl::PointXYZ>& cloud,int & p)
{
    ifstream infile;
    infile.open(file.data());
    assert(infile.is_open());

    string s;
    pcl::PointCloud<pcl::PointXYZ> t_cloud;
    t_cloud.width = 300;
    t_cloud.height = 600;
    t_cloud.is_dense = false;
    t_cloud.points.resize(t_cloud.width*t_cloud.height);

    getline(infile,s);//1
    getline(infile,s);//2
    vector<string> v;
    SplitString(s, v," ");
    Eigen::Quaterniond q= euler2Quaternion(strToFloat(v[3]),strToFloat(v[4]),strToFloat(v[5]));
//    cout<<q.coeffs()<<endl;
    Eigen::Isometry3d t(q);
    t(0,3) = strToFloat(v[0]); t(1,3) = strToFloat(v[1]); t(2,3) = strToFloat(v[2]);
    //pose-t


    int p_size = 0;
    while(getline(infile,s))
    {
        vector<string> v;
        SplitString(s, v," ");
            t_cloud.points[p_size].x = strToFloat(v[3]);
            t_cloud.points[p_size].y = strToFloat(v[4]);
            t_cloud.points[p_size].z = strToFloat(v[5]);
            p_size++;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::transformPointCloud( t_cloud, *temp, t.matrix() );
    for(int i=0;i<temp->points.size();i++){
        cloud.points[p].x = temp->points[i].x;
        cloud.points[p].y = temp->points[i].y;
        cloud.points[p].z = temp->points[i].z;
        p++;
    }
    infile.close();
}

void showAllFiles( const char * dir_name ,pcl::PointCloud<pcl::PointXYZ>& cloud,int & p)
{
    struct stat s;
    lstat( dir_name , &s );
    if( ! S_ISDIR( s.st_mode ) )
    {
        cout<<"dir_name is not a valid directory !"<<endl;
        return;
    }

    struct dirent * filename;    // return value for readdir()
    DIR * dir;                   // return value for opendir()
    dir = opendir( dir_name );
    if( NULL == dir )
    {
        cout<<"Can not open dir "<<dir_name<<endl;
        return;
    }

    while( ( filename = readdir(dir) ) != NULL )
    {
        // get rid of "." and ".."
        if( strcmp( filename->d_name , "." ) == 0 ||
            strcmp( filename->d_name , "..") == 0    )
            continue;
        string fname = filename->d_name;
        if( fname.substr(fname.size()-4,fname.size()).compare(".dat")==0){
            cout<<fname<<endl;
            readTxt("/home/daysun/rros/src/data/"+fname, cloud,p);
        }
    }
    cout<<"done\n";
}

int main(int argc,char *argv[])
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 900;
    cloud.height = 800;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);
    int p=0;
    showAllFiles("/home/daysun/rros/src/data",cloud,p);
    cout<<cloud.points.size()<<endl;
    pcl::io::savePCDFileASCII("freiburg2.pcd",cloud);
    return 0;
}
