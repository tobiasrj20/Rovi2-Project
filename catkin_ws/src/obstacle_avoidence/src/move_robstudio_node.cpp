#include <iostream>
#include "ros/ros.h"
#include "SocketCommunication.hpp"
#include <string>

// Robwork
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


int main(int argc, char **argv)
{
    //ros::init(argc, argv, "robstudio_mover"); // Init ROS
    //ros::NodeHandle nh; // Node handler

    /*
    *   Fra anden fil
    */
    string Qstring;
    SocketCommunication mySocket;
    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);

    Qstring.clear();
    for (int i = 0; i < 6 ; i++){
        Qstring.append(std::to_string(from[i]));
        Qstring.append(",");
    }

        Qstring.append("0");
        Qstring.append(",");
        Qstring.append("-0.175");
        Qstring.append(",");
        Qstring.append("1.");
        Qstring.append(",");

    cout << Qstring << endl;

    mySocket.createClient(50000);
    mySocket.sendM(Qstring);
}
