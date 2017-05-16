#include "PathPlanner_ALTO.hpp"
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
    //obstacleMotions = readMotionFile(OBSTACLE_MOTION_FILE_PATH);
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


    MovableFrame* obstacle = (MovableFrame*) wcell->findFrame("Obstacle");
    obstacle->moveTo(t_matrix,state);
    //getRobWorkStudio()->setState(state);
}

QPath PathPlanner_ALTO::getPath(rw::math::Q from, rw::math::Q to, double extend, int maxtime){

    // Get default state of the scene and move to the ''from'' position

    device->setQ(from,state);

    //moveObstacle(0, -0.175, 0.95);

    CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

    /** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner;
    Timer t;

    if (checkCollisions(device, state, detector, from))
        return 0;
    if (checkCollisions(device, state, detector, to))
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

bool PathPlanner_ALTO::checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
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
        return true;
    }
    return false;
}

int PathPlanner_ALTO::preChecker(Q ballPosition, int presentIndex){

    //moveObstacle(ballPosition[0], ballPosition[1], ballPosition[2]);

    CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    for(uint i = presentIndex; i<workingPath.size(); i++) {
        if(checkCollisions(device, state, detector, workingPath[i])) {
            return i-1;
        }
    }
    return -1;
}

QPath PathPlanner_ALTO::onlinePlanner2(uint limit, int minimumThreshold)
{
    cout << "HER!!!!!!!" << endl;
    QPath newPath;
    // push first collision free part of workingPath into tempPath
    for(uint i = 0; i < limit; i++){
        newPath.push_back(workingPath[i]);
    }

    // moveObstacle(ballPosition[0], ballPosition[1], ballPosition[2]);
    CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());

    int i = limit + 1;
    int cnt = 0;
    bool col;

    // check for colliding states with a threshold for intermediate clear states
    do {
        i++;
        if(!(col = checkCollisions(device, state, detector, workingPath[i]))){
            cnt++;
        }
        else {
            cnt = 0;
        }
    }
    while((col || cnt < minimumThreshold) && i < workingPath.size()-1);

    // if end of vector
    if(i > workingPath.size()-2){
        i = workingPath.size()-1;
    }
    else{
        i = i - (minimumThreshold - 1);
    }

    QPath bypass = getPath(workingPath[limit], workingPath[i], 0.9, 10);  // Vi skal have indstillet epislon og max time

    for(uint k = 0; k < bypass.size(); k++){
        newPath.push_back(bypass[k]);
        cout << bypass[k] << "   ny" << endl;
    }


    // push remaining part of workingPath into tempPath
    for(uint j = i+1; j < workingPath.size(); j++){
        newPath.push_back(workingPath[j]);
    }
    cout << "Workingpath internal size: " << workingPath.size() << endl;
    workingPath = newPath;
    return workingPath;
}

/*
*   Generates a new path if the ball becomes a obstacle
*/
QPath PathPlanner_ALTO::onlinePlanner(Q ballPosition)
{
    Q preCollision;
    Q postCollision;
    bool collision = false;
    QPath newPath;

    moveObstacle(ballPosition[0], ballPosition[1], ballPosition[2]);
    CollisionDetector detector(wcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    workingPath = mainPath;


    for(unsigned int i = 0; i<workingPath.size(); i++)
    {
        bool col = checkCollisions(device, state, detector, workingPath[i]);

        if(col)
        {
            if(!collision)
            {
                collision = true;
                preCollision = workingPath[i - 1];
            }
        }
        else if(!col && collision)
        {
            collision = false;
            postCollision = workingPath[i];

            QPath bypass = getPath(preCollision, postCollision, 0.9, 10);  // Vi skal have indstillet epislon og max time

            for(uint k = 0; k < bypass.size(); k++)
            {
                newPath.push_back(bypass[k]);
            }
        }
        else
        {
            newPath.push_back(workingPath[i]);
        }
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
