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
#include "Testbench.hpp"

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
    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);
    Q to(6,1.701,-0.081,0.664,3.358,-0.125,-3.314);

    PathPlanner_ALTO globalPlanner(wcFile, deviceName);

    // Generate new interpolated path
    QPath path = globalPlanner.getInterpolatedPath(from, to, 0.5, 10, 0.1);
    globalPlanner.writePathToFile(path, "../src/interpolated_mainpath.txt");

    // Read main and ball path
    //QPath mainPath = globalPlanner.readMainPathFromFile("../src/main_path4.txt");
    QPath mainPath = globalPlanner.readMainPathFromFile(argv[1]);

    QPath ballPath = globalPlanner.readBallPathFromFile("../src/ball_spiral.txt");

    // Determine robot period
    Testbench periodtest;
    periodtest.storePathLength(mainPath,"main");


    // OBS obstacle navn er ikke ændret i planner
    //string wcFile, string deviceName, string dynamicObstacleName, QPath mainPath, QPath obstaclePath, int obstaclePeriod, int devicePeriod
    ObstacleAvoidance obstacleavoidance(wcFile, deviceName, "Obstacle", mainPath, ballPath, 1000, periodtest.robotPeriod);

    obstacleavoidance.startObstacleMovement();
    while(1)
    {
        obstacleavoidance.setMainPath(mainPath);
        obstacleavoidance.runWithSimulation();
    }

}
