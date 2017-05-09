#include <iostream>
#include "ros/ros.h"
#include "SocketCommunication.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <vector>

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

// Global variables
SocketCommunication mySocket;

void qReceivedCallback(const std_msgs::Float64MultiArray msg)   //::ConstPtr&
{
    string Qstring;

    if(msg.data.size() == 9)
    {
        for(unsigned int i = 0; i<msg.data.size(); i++)
        {
            Qstring.append(std::to_string(msg.data[i]));
            Qstring.append(",");
        }

        mySocket.sendM(Qstring);
    }
    else
        cout << "Wrong message size received!" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robstudio_mover_server"); // Init ROS
    ros::NodeHandle nh; // Node handler
    ros::Subscriber sub = nh.subscribe("q_value_sender", 1, qReceivedCallback);   // Subscribe on the q_value_sender

    mySocket.createClient(50000);   // Create socket tunnel to RobworkStudio plugin

    ros::spin();
    return 0;
}
