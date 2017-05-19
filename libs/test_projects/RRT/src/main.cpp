#include <iostream>
#include "RRT.hpp"
#include "EdgeCollisionDetectors.hpp"

// Robwork libs
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/rw.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>

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

int main()
{
    const string wcFile = "../../../../Workcell3/WC3_Scene.wc.xml";
    const string deviceName = "UR1";

    WorkCell::Ptr wcell = WorkCellLoader::Factory::load(wcFile);
    Device::Ptr device = wcell->findDevice(deviceName);
    State state = wcell->getDefaultState();

    CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);
    Q to(6,1.701,-0.081,0.664,3.358,-0.125,-3.314);


    RRT rrt(constraint, sampler, metric, 0.5);

    QPath path = rrt.rrtConnectPlanner(from, to, 0.5, 1000);

    for(uint i = 0; i<path.size(); i++)
        cout << path[i] << endl;

    /*
    rrt.connectTest();

    EdgeCollisionDetectors edgeDetect(constraint, 0.1);
    edgeDetect.inCollisionBinaryTest();
    */
}
