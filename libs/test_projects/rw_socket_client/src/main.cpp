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


int main(int argc, char** argv) {
    const string wcFile = "../../../../Workcell3/WC3_Scene.wc.xml";
    const string deviceName = "UR1";

    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);
    Q to(6,1.701,-0.081,0.664,3.358,-0.125,-3.314);
    //Q ballPosition(3,-0.40,0.20,0.90);
    Q ballPosition;

    PathPlanner_ALTO planner(wcFile, deviceName);


    //Mypath.writePathToFile(path, "../src/main_path.txt");

    planner.readMainPathFromFile("../src/main_path.txt");
    QPath ballPath = planner.readBallPathFromFile("../src/ball_path.txt");

    //Transport transport(planner.getMainPath(), 500); // Create transport object for handling transmission of paths to the robot/simulator
    //transport.updateBallPos(ballPosition);

    Transport transport(planner.getMainPath(), 500, ballPath, 500); // Create transport object for handling of robot and ball movement in the simulator

    QPath correctionPath;

    int currentIndex = 0;

    while((currentIndex = transport.getCurrentIndex()) < correctionPath.size()-1){
    //for (uint i = 0; i < 2; i++){

        // Move the ball
        ballPosition = transport.getBallPosition();
        planner.moveObstacle(ballPosition[0], ballPosition[1], ballPosition[2]);

        //int currentIndex = transport.getCurrentIndex();
        cout << "current index:  " << currentIndex << endl;



        int limit = planner.preChecker(ballPosition, currentIndex);

        cout << "limit:   " << limit << endl;

        if(limit >= 0){
           transport.setLimit(limit);

           correctionPath = planner.onlinePlanner2(limit,1);

           transport.updatePath(correctionPath);
           //transport.updateBallPos(ballPosition);

           planner.printPath(correctionPath);
        }

    }

    while(1);
    cout << "Program done." << endl;
    return 0;
}
