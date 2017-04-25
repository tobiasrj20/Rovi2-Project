#pragma once
#include <string>
#include <iostream>
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

#define MAXTIME 1000.

class findPathWithEpsilon
{
  private:
    bool checkCollisions(Device::Ptr &device, const State &state, const CollisionDetector &detector, const Q &q);
    string wcFile;
    string deviceName;
    WorkCell::Ptr wc;
    Device::Ptr device;
    State state;
    CollisionDetector *detector;
    PlannerConstraint constraint;
    QSampler::Ptr sampler;
    QMetric::Ptr metric;

  public:
    findPathWithEpsilon(string wcFile, string deviceName);
    QPath findPath(double epsilon, Q from, Q to, double &time);
};
