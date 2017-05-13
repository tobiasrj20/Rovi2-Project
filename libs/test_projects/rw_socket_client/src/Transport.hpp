#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <thread>
#include <mutex>
#include <chrono>
#include "SocketCommunication.hpp"

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


class Transport
{
    private:
        SocketCommunication mySocket;
        std::mutex mtx;
        std::mutex mtx1;
        QPath workingPath;
        Q ballPosition;
        int period;
        void sendToSimulator(Q q, Q ballPosition);
        int limit = -1;
        uint currentIndex = 0;

    public:
        Transport(QPath workingPath, int period);
        void updatePath(QPath workingPath);
        void updateBallPos(Q ballPosition);
        void setLimit(int limit);
        uint getCurrentIndex();
        void transportThread();
};
