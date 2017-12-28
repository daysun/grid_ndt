#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
using namespace std;

char dir[100] = "/home/daysun/rros/src/data";
int const MAX_STR_LEN = 200;

void showAllFiles( const char * dir_name )
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

    /* read all the files in the dir ~ */
    while( ( filename = readdir(dir) ) != NULL )
    {
        // get rid of "." and ".."
        if( strcmp( filename->d_name , "." ) == 0 ||
            strcmp( filename->d_name , "..") == 0    )
            continue;
        string fname = filename->d_name;
        if( fname.substr(fname.size()-3,fname.size()).compare(".3d")==0)
            cout<<fname<<endl;
    }
    cout<<"done\n";
}

int main()
{
    showAllFiles( dir );

    return 0;
}
