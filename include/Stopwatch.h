#ifndef _WIN32
#include <sys/time.h>
#include<iostream>
#include<string>
#include <sstream>
#include <cmath>
#include "Vec3.h"
using namespace std;


string stringAndFloat(string a, float b){
    ostringstream oss;
       oss << a << b << endl;
       return oss.str() ;
}

int strToInt(string s){
    int num;
       stringstream ss(s);
       ss >> num;
       return num;
}

double stopwatch()
{
	struct timeval time;
	gettimeofday(&time, 0 );
	return 1.0 * time.tv_sec + time.tv_usec / (double)1e6;
}

int decToBinary(int n){
    string s;//result
    for(int a = n; a ;a = a/2)
    {
        s=s+(a%2?'1':'0');
    }
    std::reverse(s.begin(),s.end());
    return strToInt(s);
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
    stringstream ss;
   ss<<k;
   string str = ss.str();
   return str;
}

int binToDec(int k){
    stringstream ss;
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
int countMorton(int a,int b){
    string line = intToString(decToBinary(a));
    string column = intToString(decToBinary(b));
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
    stringstream ss;
   ss<<reverse;
   string str = ss.str();
    string result(str.rbegin(),str.rend());
// cout<<result<<endl;
    int res = binToDec(result);
//     cout<<res<<endl;
    return  res;
}

int reverseToDec(char * reverse){
    stringstream ss;
   ss<<reverse;
   string str = ss.str();
    string result(str.rbegin(),str.rend());
    int res = binToDec(result);
    return res;
}

int ToDec(char * reverse){
    stringstream ss;
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

