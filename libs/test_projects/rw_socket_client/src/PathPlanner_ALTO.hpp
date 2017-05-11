#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
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

        QPath getPath(rw::math::Q to, rw::math::Q from, double extend, int maxtime);
        void printPath();
        bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q);
        vector<Transform3D<double>> readMotionFile(std::string fileName);
        void moveObstacle();
        void moveObstacle(double x, double y, double z);
        void writePathToFile(QPath &path, std::string filepath);
        void readPathFromFile(QPath &path, std::string filepath);

        void writeMainPathToFile(std::string filepath);
        void readMainPathFromFile(std::string filepath);

    private:

        WorkCell::Ptr wcell;
        Device::Ptr device;
        QPath path;
        QPath mainPath;
        QPath workingPath;
        QToQPlanner::Ptr planner;
        MovableFrame* obstacle;
        State state;
        Timer t;
        std::vector<Transform3D<double>> obstacleMotions;
        unsigned int motionCounter;

};
