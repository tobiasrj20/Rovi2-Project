#include "PathPlanner_ALTO.hpp"
#define OBSTACLE_MOTION_FILE_PATH   "motions.txt"

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
    obstacleMotions = readMotionFile(OBSTACLE_MOTION_FILE_PATH);
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



QPath PathPlanner_ALTO::getPath(rw::math::Q to, rw::math::Q from, double extend, int maxtime){

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

    if (!checkCollisions(device, state, detector, from))
        cout << "HEY! - from er i kollision" << endl;
    if (!checkCollisions(device, state, detector, to))
        cout << "HEY! - to er i kollision" << endl;

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

void PathPlanner_ALTO::printPath(){
    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        cout << *it << endl;
    }
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
        return false;
    }
    return true;
}

/*
*   Moves the obstacle
*/
void PathPlanner_ALTO::moveObstacle()
{
    if(motionCounter >= obstacleMotions.size()) // size() -1 ???
        motionCounter = 0;
    else
        motionCounter++;

    obstacle->moveTo(obstacleMotions[motionCounter], state);
    //getRobWorkStudio()->setState(state);
}


/*
*   Reads a file with marker movements with (x,y,z,r,p,y) on each line
*   and converts each line/move to a transformation matrix
*   Returns a vector of all the transformation matrixes
*/
vector<Transform3D<double>> PathPlanner_ALTO::readMotionFile(std::string fileName)
{
    std::string line;
    double x, y, z, roll, pitch, yaw;
    RPY<double> rpy;
    Vector3D<double> xyz;
    std::vector<Transform3D<double> > motions; // Vector of transformations (rpy and translation)

    std::ifstream file(fileName);

    if(file.is_open())
    {
        // Read each line of the file
        while(std::getline(file,line))
        {
            std::stringstream lineStream(line); // Create a stream for the line string
            //std::cout << line << std::endl;
            // Read x,y,z,r,p,y from the line string
            lineStream >> x;
            lineStream >> y;
            lineStream >> z;
            lineStream >> roll;
            lineStream >> pitch;
            lineStream >> yaw;

            rpy = RPY<double>(roll,pitch,yaw);   // Create RPY matrix
            xyz = Vector3D<double>(x,y,z);   // Create translation vector

            // Create a transformation matrix from the RPY and translation vector and put it into a vector
            motions.push_back(Transform3D<double>(xyz, rpy.toRotation3D() ));
        }

        file.close();
    }

    return motions;
}


void PathPlanner_ALTO::writeMainPathToFile(std::string filepath)
{
    ofstream myfile;
    myfile.open(filepath);
    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        myfile << (*it)[0] << "," << (*it)[1] << "," << (*it)[2] << "," << (*it)[3] << "," << (*it)[4] << "," << (*it)[5] << "\n";
    }
    myfile.close();

}

void PathPlanner_ALTO::readMainPathFromFile(std::string filepath) {

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
    else cout << "fooey\n";



    for (QPath::iterator it = mainPath.begin(); it < mainPath.end(); it++) {
        cout << *it << endl;
    }
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

void PathPlanner_ALTO::readPathToFile(QPath &path, std::string filepath)
{
    ofstream myfile;
    myfile.open(filepath);
    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        myfile << (*it)[0] << "," << (*it)[1] << "," << (*it)[2] << "," << (*it)[3] << "," << (*it)[4] << "," << (*it)[5] << "\n";
    }
    myfile.close();

}
