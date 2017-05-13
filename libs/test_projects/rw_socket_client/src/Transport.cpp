#include "Transport.hpp"

Transport::Transport(QPath workingPath, int period)
{
    mySocket.createClient(50000);
    this->workingPath = workingPath;
    this->period = period;
    ballPosition = Q(3,-2,0,0);  // Initialize ballPosition
    lastBallPosition = ballPosition;
    lastRobotConfig = workingPath[0];
    std::thread *t1 = new thread(&Transport::transportThread, this);
}

Transport::Transport(QPath workingPath, int period, QPath ballPath, int ballPeriod)
{
    mySocket.createClient(50000);
    this->workingPath = workingPath;
    this->period = period;
    this->ballPeriod = ballPeriod;
    this->ballPath = ballPath;
    ballPosition = ballPath[0];  // Initialize ballPosition
    lastBallPosition = ballPosition;
    lastRobotConfig = workingPath[0];
    std::thread *t1 = new thread(&Transport::transportThread, this);
    std::thread *t2 = new thread(&Transport::ballMoveThread, this);
}

void Transport::transportThread()
{
    while(1)
    {
        // workingPath.size() is not protected and may change while reading!!
        for(uint i = 0; i<workingPath.size(); i++)
        {
            //cout << "Jeg er en traad!" << endl; // Debug
            mtx.lock();
                if(i >= limit){
                    i = limit;
                }
                else{
                    //sendToSimulator(workingPath[i], ballPosition);
                    sendToRobConfigToSimulator(workingPath[i]);
                }
                currentIndex = i;
            mtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(period));

        }
    }
}


void Transport::ballMoveThread()
{
    while(1)
    {
        for(uint i = 0; i<ballPath.size(); i++)
        {
            mtx.lock();
                sendBallPosToSimulator(ballPath[i]);
            mtx.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(ballPeriod));
        }
    }
}

void Transport::updatePath(QPath workingPath)
{
    mtx.lock();
        this->workingPath = workingPath;
        limit = -1;
    mtx.unlock();
}

void Transport::updateBallPos(Q ballPosition)
{
    mtx.lock();
        this->ballPosition = ballPosition;
    mtx.unlock();
}

void Transport::setLimit(int limit)
{
    mtx.lock();
        this->limit = limit;
    mtx.unlock();
}

uint Transport::getCurrentIndex()
{
    //mtx.lock();
        return currentIndex;
    //mtx.unlock();
}


// Send both robot q and ball position
void Transport::sendToSimulator(Q q, Q ballPosition)
{
    string Qstring;

    // Append Q
    for (uint i = 0; i < q.size(); i++){
        Qstring.append(std::to_string(q[i]));
        Qstring.append(",");
    }

    // Append obstacle movement (xyz)
    Qstring.append(std::to_string(ballPosition[0]));
    Qstring.append(",");
    Qstring.append(std::to_string(ballPosition[1]));
    Qstring.append(",");
    Qstring.append(std::to_string(ballPosition[2]));

    while(Qstring.length() < 100){
        Qstring.append("0");
    }
    // cout << "Qstring: " << Qstring << endl; // debug
    // Send robot and obstacle state
    mySocket.sendM(Qstring);
}


// Send only ball position and then the last robot config
void Transport::sendBallPosToSimulator(Q ballPosition)
{
    lastBallPosition = ballPosition;
    sendToSimulator(lastRobotConfig, ballPosition);
}

// Send only robot config and then the last ball position
void Transport::sendToRobConfigToSimulator(Q q)
{
    lastRobotConfig = q;
    sendToSimulator(q, lastBallPosition);
}

void Transport::setBallPeriod(int period)
{
    ballPeriod = period;
}

Q Transport::getBallPosition()
{
    Q ballPosision;

    mtx.lock();
        ballPosition = lastBallPosition;
    mtx.unlock();

    return ballPosition;
}
