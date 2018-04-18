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
using namespace std;

//turn data(*.* with xyz data) into pcd file in a dictionary
//maybe the size should changed
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

void readTxt(string file,pcl::PointCloud<pcl::PointXYZ>& cloud,int & p)
{
    ifstream infile;
    infile.open(file.data());
    assert(infile.is_open());

    string s;
    for(int j=0;j<1;j++){
        getline(infile,s);//1
    }

    while(getline(infile,s))
    {
        vector<string> v;
        SplitString(s, v," ");
            cloud.points[p].x = strToFloat(v[0]);
            cloud.points[p].y = strToFloat(v[1]);
            cloud.points[p].z = strToFloat(v[2]);
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
    cloud.width = 1300;
    cloud.height = 400;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);
    int p=0;
    showAllFiles("/home/daysun/rros/src/data",cloud,p);
    cout<<cloud.points.size()<<endl;
    pcl::io::savePCDFileASCII("motala52_4.pcd",cloud);
    return 0;
}
