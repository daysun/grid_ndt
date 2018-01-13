#ifndef ROBOT_H
#define ROBOT_H
#include "Vec3.h"
#include "Stopwatch.h"
#define ROBOT_TAKEUP_CELL 1   //suppose the robot take up 1*1 cells
using namespace std;

class RobotSphere{
    float r; //radius
    Vec3 position;
    Vec3 goal;
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

    //fr_23tri 0.5 0.5 --slope 0.4
    //30.02865,1.2212,0.40626
    //15.02865,-65.2212,1.3026

    //site125.pcd
    //motala52_4.pcd

    RobotSphere(const float rr, Vec3 pos= Vec3(30.02865,1.2212,0.40626),
                Vec3 goal=Vec3(63.02865,-37.2212,1.3026)):r(rr),position(pos),goal(goal){
    }
    float getRobotR() {return r;}
//    float getR(){return r/ROBOT_TAKEUP_CELL;} //get cell radius
    Vec3 getPosition(){return position;}
    Vec3 getGoal(){return goal;}
    float getReachableHeight(){
        reachableHeight = r;
        return reachableHeight;}
    float getRough(){
        //should be changed
        return 100000;
    }
    float getAngle(){
        return 60;
    }
    void setPos(string s){
        vector<string> v;
        SplitString(s, v,",");
        position.x = strToFloat(v[0]);
        position.y = strToFloat(v[1]);
        position.z = strToFloat(v[2]);
    }
    void setGoal(string s){
        vector<string> v;
        SplitString(s, v,",");
        goal.x = strToFloat(v[0]);
        goal.y = strToFloat(v[1]);
        goal.z = strToFloat(v[2]);
    }

    };


#endif // ROBOT_H
