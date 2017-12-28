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
#include"Stopwatch.h"
#include <algorithm>

using namespace std;

//test something
int main(){
//    countMorton(2,7);
    int a,b;
    mortonToXY(a,b,countMorton(83,16));
    cout<<a<<","<<b<<endl;
    mortonToXY(a,b,countMorton(94,10));
    cout<<a<<","<<b<<endl;
    mortonToXY(a,b,countMorton(87,8));
    cout<<a<<","<<b<<endl;
    mortonToXY(a,b,countMorton(82,16));
    cout<<a<<","<<b<<endl;
    cout<<strToInt("2314")<<endl;
    cout<<strToInt("2456")<<endl;
    return 1;
}
