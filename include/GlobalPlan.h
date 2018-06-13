#ifndef GLOBALPLAN_H
#define GLOBALPLAN_H
#include "map2D.h"
#include<queue>
//#include "Vec3.h"
using namespace std;
using namespace daysun;

struct MyCompare {
  bool operator()( float k1,  float k2) {
      return  k1<k2;
  }
};

class AstarPlanar{
    multimap<float,Slope *,MyCompare> open_queue;
    list<Slope *> closed_list;
    list<Slope *> global_path;
    Vector3 start,goal;
    bool isContainedClosed(Slope * s,list<Slope *> & closed_list){
        list<Slope *>::iterator it = closed_list.begin();
        while(it != closed_list.end()){
            if((*it)->morton_xy.compare(s->morton_xy)==0 && ((*it)->morton_z == s->morton_z) )
                return true;
            it++;
        }
        return false;
    }

    bool isContaninedOpen(Slope * s,multimap<float,Slope *,MyCompare>  & open_queue,
                          multimap<float,Slope *,MyCompare>::iterator & iTemp){
        multimap<float,Slope *,MyCompare>::iterator it = open_queue.begin();
        while(it != open_queue.end()){
            if((it->first) != s->f)
                break;
            if((it->second)->morton_xy.compare(s->morton_xy)==0 && ((it->second)->morton_z==s->morton_z) ){
                iTemp = it;
                return true;
            }
            it++;
        }
        return false;
    }

public:
    AstarPlanar(Vector3 start,Vector3 goal):start(start),goal(goal){
    }

    bool findRoute(TwoDmap & map2D,RobotSphere & robot,string demand){
        double time_start3 = stopwatch();
        //find where the start is
        string morton_xy,g_xy;
        int g_z;
        int morton_z;
        bool route  = false;
        map2D.transMortonXYZ(start,morton_xy,morton_z);
        map2D.transMortonXYZ(goal,g_xy,g_z);
        map<string,Cell *>::iterator it = map2D.map_cell.find(morton_xy);
        if(it != map2D.map_cell.end()){
            map<int,Slope *,CmpByKeyUD>::iterator ss = (it->second)->map_slope.find(morton_z);
            if(ss != (it->second)->map_slope.end()){
                ss->second->g = 0; //start
                ss->second->f = ss->second->g + ss->second->h;
                open_queue.insert(make_pair(ss->second->f,ss->second));

                while(open_queue.size() !=0){
                    multimap<float,Slope *,MyCompare>::iterator it_Open = open_queue.begin();
                    Slope * temp = it_Open->second; //min f
                    //if find the goal
                    if(temp->morton_xy.compare(g_xy)==0  &&
                            (temp->morton_z == g_z)){
                        route = true;
//                        cout<<"find end "<<temp->morton_xy<<","<<temp->morton_z<<endl;
                        global_path.push_front(temp);
                        cout<<"found the route to goal\n";
                        break;
                    }
                    //find neighbors
                    float tmp=2.5;
                    if(demand.compare("true") == 0){
                        tmp=4;
                    }
                    list<Slope *> neiSlope = map2D.AccessibleNeighbors(temp,robot,tmp);
                    list<Slope *>::iterator itS = neiSlope.begin();
//                    //for each traversible neighbors
                    while(itS != neiSlope.end()){
                        multimap<float,Slope *,MyCompare>::iterator  itTemp;
                            if(isContainedClosed(*itS,closed_list) || (*itS)->h == FLT_MAX){
                                //do nothing
//                                cout<<"contained in CLOSED || H= "<<(*itS)->h <<endl;
                            }
                            else if(isContaninedOpen(*itS,open_queue,itTemp)){
                                ///test
                                 Vector3 q,itn;
                                 ///use the morton_xyz
//                                string s_xy = temp->morton_xy;
//                                string s_z = temp->morton_z;
//                                map2D.countPositionXYZ(q.x,q.y,q.z,s_xy,s_z);
//                                string n_xy = (*itS)->morton_xy;
//                                string n_z = (*itS)->morton_z;
//                                map2D.countPositionXYZ(itn.x,itn.y,itn.z,n_xy,n_z);
                                ///use the mean
                                q = temp->mean;
                                itn = (*itS)->mean;

                                if((*itS)->g > temp->g + map2D.TravelCost(q,itn)){
                                    //remove and insert
                                    //update g and f, and father node
//                                    cout<<"itS g"<<(*itS)->g<<endl; ///dont know if its right
//                                    cout<<"itemp g "<<(itTemp->second)->g<<endl;
                                    (*itS)->g = temp->g + map2D.TravelCost(q,itn);
                                    (*itS)->f = (*itS)->g + (*itS)->h;
                                    (*itS)->father = temp;
                                    open_queue.erase(itTemp);
                                    open_queue.insert(make_pair( (*itS)->f ,*itS));
                                }
                            }
                            else{
                                //update g and f, and father node
                                //insert into OPEN
                                ///test
                                 Vector3 q,itn;
                                 ///use the morton_xyz
//                                string s_xy = temp->morton_xy;
//                                string s_z = temp->morton_z;
//                                map2D.countPositionXYZ(q.x,q.y,q.z,s_xy,s_z);
//                                string n_xy = (*itS)->morton_xy;
//                                string n_z = (*itS)->morton_z;
//                                map2D.countPositionXYZ(itn.x,itn.y,itn.z,n_xy,n_z);
                                 ///use the mean
                                 q = temp->mean;
                                 itn = (*itS)->mean;

                                 (*itS)->g = temp->g + map2D.TravelCost(/*temp->mean,(*itS)->mean*/q,itn);
                                 (*itS)->f = (*itS)->g + (*itS)->h;
                                 (*itS)->father = temp;
                                 open_queue.insert(make_pair( (*itS)->f ,*itS));
                            }
                        itS++;
                    }
                    closed_list.push_back(temp);
                    open_queue.erase(it_Open);
                }
            }else{
                cout<<"1,Sth wrong with the start slope, cant find it.\n";
             return false;
            }
        }else{
            cout<<"2,Sth wrong with the start slope, cant find it.\n";
             return false;
        }

        if(route){
            Slope * i = global_path.front();
            while(i->father != NULL){
                global_path.push_front(i->father);
                i = global_path.front();
            }
            double time_end3 = stopwatch();
            cout<<"Global planr done. A*: "<<(time_end3-time_start3)<<" s\n";
            return true;
        }else{
            cout<<"not find the road\n";
            return false;
        }
    }

    void showRoute(TwoDmap & map2D,ros::Publisher marker_pub){
        map2D.showSlopeList(marker_pub,global_path,4);
        cout<<"route show done\n";
    }

};
#endif // GLOBALPLAN_H
