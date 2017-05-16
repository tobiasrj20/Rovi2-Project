#include "ObstacleAvoidance.hpp"
#include "Testbench.hpp"

ObstacleAvoidance::ObstacleAvoidance(const string wcFile, const string deviceName, const string dynamicObstacleName, const QPath mainPath, const QPath obstaclePath, const int obstaclePeriod, const int devicePeriod)
{
    this->mainPath = mainPath;
    this->obstaclePath = obstaclePath;
    this->planner = new PathPlanner_ALTO(wcFile, deviceName);
    this->transport = new Transport(mainPath, devicePeriod, obstaclePath, obstaclePeriod); // Create transport object for handling of robot and ball movement in the simulator
}


void ObstacleAvoidance::setMainPath(QPath mainPath)
{
    this->mainPath = mainPath;
    planner->setMainPath(mainPath);
    planner->setWorkingPath(mainPath);
    transport->updatePath(mainPath);
}

void ObstacleAvoidance::startObstacleMovement()
{
    transport->startBallThread();
}

void ObstacleAvoidance::runWithSimulation()
{
    Q ballPosition;
    QPath correctionPath;
    int currentIndex = 0;
    int limit;
    Testbench testbench;

    transport->startRobotThread();

    // Wait until thread is ready
    while(transport->getCurrentIndex() == -1);   //  Ugly fix to secure that thread is running once, before the while loop below.

    while((currentIndex = transport->getCurrentIndex()) > -1)
    {

        // Move the ball
        ballPosition = transport->getBallPosition();
        planner->moveObstacle(ballPosition[0], ballPosition[1], ballPosition[2]);

        cout << "current index:  " << currentIndex << endl;
        testbench.startPreCheckTime();
        limit = planner->preChecker(ballPosition, currentIndex);
        testbench.pausePreCheckTime();
        cout << "Precheck time: " << testbench.precheck_time << endl;
        testbench.precheck_time = 0;

        cout << "limit:   " << limit << endl;

        if(limit >= 0){
           transport->setLimit(limit);

           testbench.startTimer();
           correctionPath = planner->onlinePlanner2(limit,1);
           testbench.stopTimer();
           transport->updatePath(correctionPath);
        }
    }
    planner->printPath(correctionPath);
    //cout << "OMGANG!!!" << endl;
    //transport->deleteThread();
}

ObstacleAvoidance::~ObstacleAvoidance()
{
    delete planner;
    delete transport;
}
