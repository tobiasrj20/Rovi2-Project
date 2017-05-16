#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>

#include <rwlibs/pathplanners/z3/Z3Planner.hpp>
#include <rwlibs/pathplanners/z3/Z3QToQPlanner.hpp>

#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>
#include <rwlibs/pathplanners/arw/ARWQToQPlanner.hpp>

#include <rwlibs/pathplanners/sbl/SBLSetup.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLOptions.hpp>

#include <rwlibs/pathplanners/prm/PRMPlanner.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include "Transport.hpp"
#include "PathPlanner_ALTO.hpp"

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

class ObstacleAvoidance
{
    public:
        ObstacleAvoidance(const string wcFile, const string deviceName, const string dynamicObstacleName, const QPath mainPath, const QPath obstaclePath, const int obstaclePeriod, const int devicePeriod);
        ~ObstacleAvoidance();
        void setMainPath(QPath mainPath);
        void runWithSimulation();
        void startObstacleMovement();

    private:
        PathPlanner_ALTO *planner;
        Transport *transport;
        QPath mainPath;
        QPath obstaclePath;

};
