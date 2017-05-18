#include <boost/bind.hpp>
#include <iostream>
#include <QTime>
#include <QCoreApplication>
#include <unistd.h>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <fstream>
#include "PathPlanner_ALTO.hpp"
#include "Transport.hpp"
#include <thread>
#include "ObstacleAvoidance.hpp"

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

double getJointLength(QPath path){
    double sum = 0;
    Q diff;
    for(unsigned int i = 1; i<path.size(); i++){
        diff = path[i-1] - path[i];
        sum += diff.norm2();
    }
    return sum;
}

int main(int argc, char** argv)
{
    const string wcFile = "../../../../Workcell3/WC3_Scene.wc.xml";
    const string deviceName = "UR1";

    PathPlanner_ALTO globalPlanner(wcFile, deviceName);
    QPath mainPath = globalPlanner.readMainPathFromFile("../src/main_path.txt");
    QPath ballPath = globalPlanner.readBallPathFromFile("../src/ball_path.txt");
    
    // OBS obstacle navn er ikke ændret i planner
    //string wcFile, string deviceName, string dynamicObstacleName, QPath mainPath, QPath obstaclePath, int obstaclePeriod, int devicePeriod
    ObstacleAvoidance obstacleavoidance(wcFile, deviceName, "Obstacle", mainPath, ballPath, 1000, 500);

    obstacleavoidance.startObstacleMovement();
    while(1)
    {
        obstacleavoidance.setMainPath(mainPath);
        obstacleavoidance.runWithSimulation();
    }

}
