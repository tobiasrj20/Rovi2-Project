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


class PathPlanner_ALTO
{
    public:
        PathPlanner_ALTO(const string wcFile, const string deviceName);

        QPath getPath(rw::math::Q from, rw::math::Q to, double extend, int maxtime);
        void printPath(QPath path);
        bool checkCollisions(Device::Ptr device, const State &state, /*const CollisionDetector &detector,*/ const Q &q);
        vector<Transform3D<double>> readMotionFile(std::string fileName);

        int preChecker(Q ballPosition, int presentIndex);
        QPath correctionPlanner(uint limit, int minimumThreshold);

        void moveObstacle(double x, double y, double z);
        void writePathToFile(QPath &path, std::string filepath);
        void readPathFromFile(QPath &path, std::string filepath);
        void writeMainPathToFile(std::string filepath);
        QPath readMainPathFromFile(std::string filepath);
        QPath readBallPathFromFile(std::string filepath);
        QPath getMainPath();
        void setMainPath(QPath path);
        void setWorkingPath(QPath path);

    private:
        WorkCell::Ptr wcell;
        Device::Ptr device;

        QPath mainPath;
        QPath workingPath;
        QToQPlanner::Ptr planner;
        MovableFrame* obstacle;
        State state;
        Timer t;
        std::vector<Transform3D<double>> obstacleMotions;
        unsigned int motionCounter;
        bool binaryLocalPlanner(Q to, Q from);
        CollisionDetector *detector;

};
