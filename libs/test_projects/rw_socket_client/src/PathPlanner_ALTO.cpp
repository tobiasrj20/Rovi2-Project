#include "PathPlanner_ALTO.hpp"
#include "Testbench.hpp"
//#define OBSTACLE_MOTION_FILE_PATH   "motions.txt"

PathPlanner_ALTO::PathPlanner_ALTO(const string wcFile, const string deviceName)
{
    wcell = WorkCellLoader::Factory::load(wcFile);
    device = wcell->findDevice(deviceName);
    obstacle = (MovableFrame*) wcell->findFrame("Obstacle");

    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
    }
    if (device == NULL) {
        cerr << "Obstacle not found!" << endl;
    }
    rw::math::Math::seed();

    // Get default state
    state = wcell->getDefaultState();

    // Load obstacle motion
    motionCounter = 0;

    moveObstacle(-0.595,0.000,1.717);   // Start ball position in scene
    // Create detector for collision detection
    detector = new CollisionDetector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
}

QPath PathPlanner_ALTO::getMainPath(){
   return mainPath;
}

void PathPlanner_ALTO::moveObstacle(double x, double y, double z) {
    double roll = 0.000;
    double pitch = 0.000;
    double yaw = 0.000;

    RPY<double> rpy(roll,pitch,yaw);   // Create RPY matrix
    Vector3D<double> xyz(x,y,z);   // Create translation vector
    Transform3D<double> t_matrix(xyz, rpy.toRotation3D() ); // Create a transformation matrix from the RPY and XYZ

    obstacle = (MovableFrame*) wcell->findFrame("Obstacle");
    obstacle->moveTo(t_matrix,state);
}

QPath PathPlanner_ALTO::getPath(rw::math::Q from, rw::math::Q to, double extend, int maxtime){

    // Get default state of the scene and move to the ''from'' position

    device->setQ(from,state);

    //moveObstacle(0, -0.175, 0.95);

    //CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(detector,device,state);

    /** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner;
    Timer t;

    if (checkCollisions(device, state, /*detector,*/ from))
        return 0;
    if (checkCollisions(device, state, /*detector,*/ to))
        return 0;

    // Single path generation
    QPath path;

    planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
    //planner = Z3Planner::makeQToQPlanner(constraint,device);
    //planner = ARWPlanner::makeQToQPlanner(constraint,device,0,-1,-1);

    t.resetAndResume();
    planner->query(from,to,path,maxtime);
    t.pause();

    if (t.getTime() >= maxtime) {
       cout << "Notice: max time of " << maxtime << " seconds reached." << endl;
    }

    return path;
}

void PathPlanner_ALTO::printPath(QPath path){
    cout << "size: " << path.size() << endl;
    for(uint i = 0; i<path.size(); i++)
        cout << path[i] << endl;
    /*for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        cout << *it << endl;
    }*/
}

bool PathPlanner_ALTO::checkCollisions(Device::Ptr device, const State &state, /*const CollisionDetector &detector,*/ const Q &q) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector->inCollision(testState,&data);
    if (colFrom) {
        //cerr << "Configuration in collision: " << q << endl;
        //cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            //cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return true;
    }
    return false;
}

QPath PathPlanner_ALTO::correctionPlanner(uint limit, int minimumThreshold)
{
    QPath newPath;

    CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    uint i = limit + 1;  // Index of first collision

    while(checkCollisions(device, state, /*detector,*/ workingPath[i]) && i < workingPath.size())
    {
        if(i + 1 >= workingPath.size())
            break;
        else
            i++;
    }

    QPath bypass = getPath(workingPath[limit], workingPath[i], 0.5, 10);  // Vi skal have indstillet epislon og max time

    // push first collision free part of workingPath into tempPath
    for(uint h = 0; h < limit; h++){
        newPath.push_back(workingPath[h]);
    }

    for(uint k = 0; k < bypass.size(); k++){
        newPath.push_back(bypass[k]);
        //cout << bypass[k] << "   ny" << endl;
    }

    // push remaining part of workingPath into tempPath
    for(uint j = i+1; j < workingPath.size(); j++){
        newPath.push_back(workingPath[j]);
    }
    workingPath = newPath;
    return workingPath;
}


void PathPlanner_ALTO::writeMainPathToFile(std::string filepath)
{
    ofstream myfile;
    myfile.open(filepath);
    for (QPath::iterator it = mainPath.begin(); it < mainPath.end(); it++) {
        myfile << (*it)[0] << "," << (*it)[1] << "," << (*it)[2] << "," << (*it)[3] << "," << (*it)[4] << "," << (*it)[5] << "\n";
    }
    myfile.close();

}

QPath PathPlanner_ALTO::readMainPathFromFile(std::string filepath) {

    vector<double> state_vec;
    std::string line, token;
    std::string::size_type sz;
    std::ifstream myfile;

    mainPath.clear();

    myfile.open(filepath);
    if (myfile) {
        while (getline( myfile, line )) {
            state_vec.clear();
            std::istringstream ss(line);
            while(getline(ss, token, ',')) {
                state_vec.push_back( std::stod(token,&sz));
            }
        mainPath.push_back(Q(6, state_vec[0],state_vec[1],state_vec[2],state_vec[3],state_vec[4],state_vec[5]));
        //cout << temp << endl;
        }
        myfile.close();
    }
    else
        cout << "Could not read file! " << endl;


    workingPath = mainPath;
    return workingPath;
    /*
    for (QPath::iterator it = mainPath.begin(); it < mainPath.end(); it++) {
        cout << *it << endl;
    }*/
}

QPath PathPlanner_ALTO::readBallPathFromFile(std::string filepath)
{
    vector<double> state_vec;
    std::string line, token;
    std::string::size_type sz;
    std::ifstream myfile;
    QPath ballPath;

    myfile.open(filepath);
    if (myfile) {
        while (getline( myfile, line )) {
            state_vec.clear();
            std::istringstream ss(line);
            while(getline(ss, token, ',')) {
                state_vec.push_back( std::stod(token,&sz));
            }
        ballPath.push_back(Q(3, state_vec[0],state_vec[1],state_vec[2]));

        }
        myfile.close();
    }
    else
        cout << "Could not read file! " << endl;

    return ballPath;
}


void PathPlanner_ALTO::writePathToFile(QPath &path, std::string filepath)
{
    ofstream myfile;
    myfile.open(filepath);
    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        myfile << (*it)[0] << "," << (*it)[1] << "," << (*it)[2] << "," << (*it)[3] << "," << (*it)[4] << "," << (*it)[5] << "\n";
    }
    myfile.close();

}

void PathPlanner_ALTO::setMainPath(QPath path)
{
    this->mainPath = path;
}

void PathPlanner_ALTO::setWorkingPath(QPath path)
{
    this->workingPath = path;
}


int PathPlanner_ALTO::preChecker(Q ballPosition, int presentIndex){

    for(uint i = presentIndex; i < workingPath.size(); i++) {
        if(checkCollisions(device, state, /*detector,*/ workingPath[i])) {
            return i - 1;
        }

        if(i > presentIndex){
            if(!binaryLocalPlanner(workingPath[i - 1],workingPath[i])){
               return i - 1;
           }
       }
    }
    return -1;
}

bool PathPlanner_ALTO::binaryLocalPlanner(Q to, Q from){

    State testState;
    CollisionDetector::QueryResult data;
    //CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    Q q;
    Q deltaQ = to-from;

    double epsilon = 1.0;
    int n = deltaQ.norm2()/epsilon;
    int levels = ceil(log2(n));

    for(int i = 1; i <= levels; i++){
        int steps = pow(2.0,i-1);
        Q step = deltaQ/steps;
        for(int j = 1; j <= steps; j++){

            q = from + (j - 0.5)*step;
            //cout << q << endl;
            testState = state;
            device->setQ(q,testState);
            if(detector->inCollision(testState,&data)){
                return false;
            }
        }
    }
    return true;
}
