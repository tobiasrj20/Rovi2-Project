#include <boost/bind.hpp>
#include <iostream>
#include "SocketCommunication.hpp"
#include <QTime>
#include <QCoreApplication>
#include <unistd.h>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <fstream>
#include "PathPlanner_ALTO.hpp"

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

int main(int argc, char** argv) {

    const string wcFile = "../../../../Workcell3/WC3_Scene.wc.xml";
    const string deviceName = "UR1";

    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);
    Q to(6,1.701,-0.081,0.664,3.358,-0.125,-3.314);

    PathPlanner_ALTO Mypath(wcFile, deviceName);
/*
    Mypath.moveObstacle(0, -0.175, 3.);

    QPath path = Mypath.getPath(to,from,0.9,10.);
    Mypath.printPath(path);

    Mypath.writePathToFile(path, "../src/main_path.txt");
*/
    Mypath.readMainPathFromFile("../src/main_path.txt");


/*
    vector<string> string_state_vec;
    string Qstring;

    SocketCommunication mySocket;


    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        Qstring.clear();
        for (int i = 0; i < 6 ; i++){
            Qstring.append(std::to_string((*it)[i]));
            Qstring.append(",");
        }

            Qstring.append("0");
            Qstring.append(",");
            Qstring.append("-0.175");
            Qstring.append(",");
            Qstring.append("1.");

            while(Qstring.length() < 100){
                Qstring.append("0");
            }

        string_state_vec.push_back(Qstring);
    }

    mySocket.createClient(50000);
    //mySocket.sendM(Qstring);



    for (uint j = 0; j < 100; j++){
        for (uint i = 0; i < string_state_vec.size() ; i++){
            mySocket.sendM(string_state_vec[i]);
            usleep(100000);
        }
        usleep(800000);
    }



*/
    cout << "Program done." << endl;
    return 0;
}
