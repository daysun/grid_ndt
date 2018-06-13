#ifndef ROBOT_H
#define ROBOT_H
#include "Vec3.h"
#include "Vector3.h"
#include "Stopwatch.h"
#define ROBOT_TAKEUP_CELL 1   //suppose the robot take up 1*1 cells
using namespace std;

class RobotSphere{
    float r; //radius
    Vector3 position;
    Vector3 goal;
    float reachableHeight;//height that it can reach
public:
//    list<Slope *> trajectory;
    //test
    //robot 0.25/0.5
    //gridLen 0.25
    //pos 2.75533,1.36108,1.67499
    //goal -0.52865,0.00212,1.66626
    //goal under the floor 0.02865,-0.50212,1.66626

    //fr2.pcd 0.5 0.5
    //pos  -3.75533,1.36108,-0.1499
    //goal 15.02865,1.1212,0.40626

    //fr2-16 0.5 0.5
    //30.02865,1.2212,0.40626
    //63.02865,-37.2212,1.3026

    RobotSphere(const float rr, Vector3 pos= Vector3(30.02865,1.2212,0.40626),
                Vector3 goal=Vector3(63.02865,-37.2212,1.3026)):r(rr),position(pos),goal(goal){
    }
    float getRobotR() {return r;}
//    float getR(){return r/ROBOT_TAKEUP_CELL;} //get cell radius
    Vector3 getPosition(){return position;}
    Vector3 getGoal(){return goal;}
    float getReachableHeight(){
        return 0.15;}
    float getRough(){
        //should be changed
        return 100;
    }
    float getAngle(){
        return 30;
    }
    void setPos(string s){
        vector<string> v;
        SplitString(s, v,",");
        position(0) = strToFloat(v[0]);
        position(1) = strToFloat(v[1]);
        position(2) = strToFloat(v[2]);
    }
    void setGoal(string s){
        vector<string> v;
        SplitString(s, v,",");
        goal(0) = strToFloat(v[0]);
        goal(1) = strToFloat(v[1]);
        goal(2) = strToFloat(v[2]);
    }

    };


#endif // ROBOT_H
