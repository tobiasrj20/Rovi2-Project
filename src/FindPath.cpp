#include "FindPath.hpp"

FindPath::FindPath(const string wcFile, const string deviceName)
{
    wcell = WorkCellLoader::Factory::load(wcFile);
    device = wcell->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
    }
    rw::math::Math::seed();
}

QPath FindPath::getPath(rw::math::Q to, rw::math::Q from, double extend, int maxtime){

    // Get default state of the scene and move to from
    State state = wcell->getDefaultState();
    device->setQ(from,state);

    CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

    /** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner;
    Timer t;

    if (!checkCollisions(device, state, detector, from))
        return 0;
    if (!checkCollisions(device, state, detector, to))
        return 0;

    // Single path generation
    path.clear();
    planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
    t.resetAndResume();
    planner->query(from,to,path,maxtime);
    t.pause();

    if (t.getTime() >= maxtime) {
       cout << "Notice: max time of " << maxtime << " seconds reached." << endl;
    }

    return path;
}

void FindPath::printPath(){
    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        cout << *it << endl;
    }
}

bool FindPath::checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
}
