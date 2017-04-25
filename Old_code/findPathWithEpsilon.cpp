#include "findPathWithEpsilon.hpp"
#include "rw/kinematics/Kinematics.hpp"
findPathWithEpsilon::findPathWithEpsilon(string wcFile, string deviceName)
{
  this->wcFile = wcFile;
  this->wcFile = deviceName;

  wc = WorkCellLoader::Factory::load(wcFile);
	device = wc->findDevice(deviceName);

  if (device == NULL) {
		//cerr << "Device: " << deviceName << " not found!" << endl;
		return;
	}
	state = wc->getDefaultState();
	detector = new CollisionDetector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());

	/** Most easy way: uses default parameters based on given device
		sampler: QSampler::makeUniform(device)
		metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
		extend: 0.05 */
	//QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);

	/** More complex way: allows more detailed definition of parameters and methods */
	metric = MetricFactory::makeEuclidean<Q>();
}

bool findPathWithEpsilon::checkCollisions(Device::Ptr &device, const State &state, const CollisionDetector &detector, const Q &q)
{
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

QPath findPathWithEpsilon::findPath(double epsilon, Q from, Q to, double &time)
{
    auto gripper = wc->findFrame("Tool");
    auto bottle = wc->findFrame("Bottle");
    device->setQ(from,state);
    Kinematics::gripFrame(bottle, gripper, state);
    constraint = PlannerConstraint::make(detector,device,state);
	sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, RRTPlanner::RRTConnect);

	if (!checkCollisions(device, state, *detector, from))
		return 0;
	if (!checkCollisions(device, state, *detector, to))
		return 0;

	//cout << "Planning from " << from << " to " << to << endl;
	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();
	//cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
	if (t.getTime() >= MAXTIME) {
		cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
	}
    time = t.getTime();
	return path;
}
