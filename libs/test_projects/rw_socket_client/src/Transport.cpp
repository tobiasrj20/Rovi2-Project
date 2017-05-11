#include "Transport.hpp"

Transport::Transport(QPath workingPath, int period)
{
    mySocket.createClient(50000);
    this->workingPath = workingPath;
    this->period = period;
    ballPosition = Q(3,-2,0,0);  // Initialize ballPosition
    std::thread *t1 = new thread(&Transport::transportThread, this);
}

void Transport::transportThread()
{
    while(1)
    {
        for(uint i = 0; i<workingPath.size(); i++)
        {
            cout << "Jeg er en traad!" << endl; // Debug
            mtx.lock();
                sendToSimulator(workingPath[i], ballPosition);
            mtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(period));

        }
    }
}

void Transport::updatePath(QPath workingPath)
{
    mtx.lock();
        this->workingPath = workingPath;
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

}

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
    cout << "Qstring: " << Qstring << endl;
    // Send robot and obstacle state
    mySocket.sendM(Qstring);
}
