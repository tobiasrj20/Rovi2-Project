#include "Testbench.hpp"

Testbench::Testbench(){
    M_P_length = 0;
    W_P_length = 0;
    precheck_time = 0;
    planning_time = 0;
    noOfCol = 0;
}

void Testbench::storePathLength(QPath path, const std::string& type){
    double sum = 0;
    Q diff;
    for(uint i = 1; i<path.size(); i++){
        diff = path[i-1] - path[i];
        sum += diff.norm2();
    }
    if (type == "main")
        M_P_length = sum;
    else if (type == "work")
        W_P_length = sum;


    double maxbetween;
    double maxtotal = 0;
    for(uint i = 1; i<path.size(); i++){
        diff = path[i-1] - path[i];
        cout << diff << endl;
        maxbetween = 0;
        for(uint j = 1; j<6; j++){
            if(maxbetween < diff[j]){
                maxbetween = diff[j];
            }
        }
        maxtotal += maxbetween;
    }

    double each = maxtotal/(path.size()-1);
    cout << "Each: " << each *1000 << "ms" << endl;

    if(each >= 0.42){
        cout << "Acceleration: " << ((2*0.42)/3.1415926535)*1000 << "ms" << endl;
        cout << "Resten: " << ((each -0.42)/3.1415926535)*1000 << "ms" << endl;
        robotPeriod = (((each-0.42)+(2*0.42))/3.1415926535)*1000;
        cout << "Robot period: " << robotPeriod << "ms" << endl;
    }
    else
    {
        robotPeriod = ((2*each)/3.1415926535)*1000;
        cout << "Robot period (only acceleration): " << robotPeriod << "ms" << endl;
    }
}


void Testbench::collisionCount(){
    noOfCol++;
}

void Testbench::startPreCheckTime(){
    startprethecktime = high_resolution_clock::now();
}

void Testbench::pausePreCheckTime(){
    time_span = duration_cast<duration<double>>( high_resolution_clock::now() - startprethecktime);
    precheck_time += time_span.count();
}


void Testbench::startPlanningTime(){
    startplanningtime = high_resolution_clock::now();
}

void Testbench::pausePlanningTime(){
    time_span = duration_cast<duration<double>>( high_resolution_clock::now() - startplanningtime);
    planning_time += time_span.count();
}

void Testbench::startTimer()
{
    startSimpleTime = high_resolution_clock::now();
}

void Testbench::stopTimer()
{
    time_span = duration_cast<duration<double>>( high_resolution_clock::now() - startSimpleTime);
    simple_time += time_span.count();
    cout << "Timer time: " << simple_time << endl;
    simple_time = 0;
}
