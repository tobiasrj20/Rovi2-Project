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
        std::mutex ballMtx;
        QPath workingPath;
        Q ballPosition;
        Q lastBallPosition;
        Q lastRobotConfig;
        QPath ballPath;
        int period;
        int ballPeriod;
        void sendToSimulator(Q q, Q ballPosition);
        void sendBallPosToSimulator(Q ballPosition);
        void sendToRobConfigToSimulator(Q q);
        int limit = -1;
        int currentIndex = -1;
        std::thread *t1;
        std::thread *t2;

    public:
        Transport();
        Transport(QPath workingPath, int period);
        Transport(QPath workingPath, int period, QPath ballPath, int ballPeriod);
        void updatePath(QPath workingPath);
        void updateBallPos(Q ballPosition);
        void setLimit(int limit);
        int getCurrentIndex();
        void transportThread();
        void ballMoveThread();
        void setBallPeriod(int period);
        Q getBallPosition();
        void startBallThread();
        void startRobotThread();
        Q getCurrentQ();
        void deleteThread();
};
