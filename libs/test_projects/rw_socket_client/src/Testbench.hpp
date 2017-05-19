#pragma once
#include <boost/bind.hpp>
#include <iostream>
#include <QTime>
#include <QCoreApplication>
#include <unistd.h>
#include <fstream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <rw/rw.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::trajectory;

using namespace std::chrono;

class Testbench
{

    public:
        Testbench();
        void startPreCheckTime();
        void pausePreCheckTime();

        void startPlanningTime();
        void pausePlanningTime();

        void collisionCount();
        void storePathLength(QPath path, const std::string& type);

        void startTimer();
        void stopTimer();

        double M_P_length;
        double W_P_length;
        int noOfCol;

        double precheck_time;
        double planning_time;
        double simple_time;
        double robotPeriod;

    private:
        high_resolution_clock::time_point startSimpleTime;
        high_resolution_clock::time_point startprethecktime;
        high_resolution_clock::time_point startplanningtime;
        duration<double> time_span;
};
