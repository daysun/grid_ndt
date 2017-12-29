#ifndef ROBOT_H
#define ROBOT_H
#include "Vec3.h"
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

    //sys
    //pos
    //goal 1.49998,0.128561,-0.0197856
    //0.954722,-2.75786,-0.777927
    //1.3711,-2.77096,-0.865337

    ///no-use
    //28_0047.bag
    //0.5 1
    //pos -6.75533,1.36108,-2.07499
    //goal -15.02865,-1.6212,-2.06626

    //vision
    //0.5 1.5
    //pos -6.75533,-1.36108,-1.07499
    //goal -75.02865,-18.6212,-1.06626

    //fr2.pcd 0.5 0.5
    //pos -7.75533,2.36108,0.07499
    //goal 15.02865,1.1212,0.56626

    RobotSphere(const float rr, Vec3 pos= Vec3(-3.75533,3.36108,0.07499),
                Vec3 goal=Vec3(15.02865,1.1212,0.56626)):r(rr),position(pos),goal(goal){
    }
    float getRobotR() {return r;}
    float getR(){return r/ROBOT_TAKEUP_CELL;} //get cell radius
    Vec3 getPosition(){return position;}
    Vec3 getGoal(){return goal;}
    float getReachableHeight(){
        reachableHeight = r/2;
        return reachableHeight;}
    float getRough(){
        //should be changed
        return 10000;
    }
    float getAngle(){
        return 30;
    }

    };


#endif // ROBOT_H
