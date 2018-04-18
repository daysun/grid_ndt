#ifndef _WIN32
#include <sys/time.h>
#include<iostream>
#include<string>
#include <sstream>
#include <cmath>
#include "Vec3.h"
#include "Vector3.h"
#include<boost/format.hpp>
using namespace std;
using namespace octomath;


string stringAndFloat(string a, float b){
    ostringstream oss;
       oss << a << b << endl;
       return oss.str() ;
}

int strToInt(string s){
    int num;
       istringstream ss(s);
       ss >> num;
       return num;
}

double stopwatch()
{
    struct timeval time;
    gettimeofday(&time, 0 );
    return 1.0 * time.tv_sec + time.tv_usec / (double)1e6;
}

//int decToBinary(int n){

//    return strToInt(s);
//}

string decToBinStr2(int n){
    string s;//result
    for(int a = n; a ;a = a/2)
    {
        s=s+(a%2?'1':'0');
    }
    std::reverse(s.begin(),s.end());
    return s;
}

string decToBinaryStr(int n){
    string s;//result
    for(int a = n; a ;a = a/2)
    {
        s=s+(a%2?'1':'0');
    }
    std::reverse(s.begin(),s.end());
    return s;
}

string intToString(int k){
    ostringstream ss;
   ss<<k;
   string str = ss.str();
   return str;
}

int binToDec(int k){
    ostringstream ss;
   ss<<k;
   string str = ss.str();
    unsigned int i = 0;
       const char *pch = str.c_str();
       while (*pch == '0' || *pch == '1') {
           i <<= 1;
           i |= *pch++ - '0';
       }
       return (int)i;
}

float strToFloat(string s){
    float num;
       istringstream ss(s);
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

int binToDec(string str){
    unsigned int i = 0;
       const char *pch = str.c_str();
       while (*pch == '0' || *pch == '1') {
           i <<= 1;
           i |= *pch++ - '0';
       }
       return (int)i;
}

//for example
//line 5: 0101 column 7:0111
//morton:        110111
//return dec:   55
string countMorton(int a,int b){
    string line = decToBinStr2(a);
    string column = decToBinStr2(b);
    if( line.size()>column.size()){
        int num = line.size()-column.size();
        for(int i =0;i<num;i++){
            column.insert(0,"0");
        }
    }else if(line.size()<column.size()){
        int num = column.size()-line.size();
        for(int i =0;i<num;i++){
            line.insert(0,"0");
        }
    }
//   cout<<line<<","<<column<<endl;
    int size = line.size() + column.size();
    char * reverse = new char[size+1];
    int j  =0;
    for(int i = line.size()-1;i>=0;i--){
        reverse[j++] = column[i];
        reverse[j++] = line[i];
    }
    reverse[size] = '\0';
//     cout<<reverse<<endl;
    ostringstream ss;
   ss<<reverse;
   string str = ss.str();
    string result(str.rbegin(),str.rend());
// cout<<result<<endl;
    string res = intToString(binToDec(result));
    return  res;
}

int reverseToDec(char * reverse){
    ostringstream ss;
   ss<<reverse;
   string str = ss.str();
    string result(str.rbegin(),str.rend());
    int res = binToDec(result);
    return res;
}

int ToDec(char * reverse){
    ostringstream ss;
   ss<<reverse;
   string str = ss.str();
    int res = binToDec(str);
    return res;
}

//count out column and line from morton_xy
//for example
//morton_xy    55
//morton_xy    110111
//return line 5: 0101 column 7:0111
void mortonToXY(int & a, int &b, int morton){
    string m = decToBinaryStr(morton);
    //add '0'
    if(m.size()%2 != 0){ //attention
        m.insert(0,"0");
    }
    int size = m.size()/2 ;
    char * column = new char[size+1];
    char * line= new char[size+1];
    int j = 0;
    for(int i=0;i<m.size();i+=2,j++){
        line[j] = m[i];
        column[j] = m[i+1];
    }
    column[size] = '\0';
    line[size] = '\0';
    a = ToDec(line);
    b = ToDec(column);
}


#else

#include <windows.h>
double stopwatch()
{
    unsigned long long ticks;
    unsigned long long ticks_per_sec;
    QueryPerformanceFrequency( (LARGE_INTEGER *)&ticks_per_sec);
    QueryPerformanceCounter((LARGE_INTEGER *)&ticks);
    return ((float)ticks) / (float)ticks_per_sec;
}

#endif

