#include "ros/ros.h"
#include <iostream>
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner"); // Init ROS
    ros::NodeHandle nh; // Node handler
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("q_value_sender", 1);    // Publisher for q-values to move_robotstudio_node

    // test
    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);
    Q to(6,1.701,-0.081,0.664,3.358,-0.125,-3.314);

    while (ros::ok())
    {
        std_msgs::Float64MultiArray msg;

        for(unsigned int i = 0; i<from.size(); i++)
        {
            msg.data.push_back(from[i]);
        }

        msg.data.push_back(0);
        msg.data.push_back(-0.175);
        msg.data.push_back(1.0);

        pub.publish(msg);
    }

    return 0;
}


/*


FindPath Mypath(wcFile, deviceName);
Mypath. moveObstacle(0, -0.175, 1);

*/
