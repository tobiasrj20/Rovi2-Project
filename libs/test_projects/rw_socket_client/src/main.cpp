#include <boost/bind.hpp>
#include <iostream>
#include "../../../SocketCommunication/SocketCommunication.hpp"
#include <QTime>
#include <QCoreApplication>
#include <unistd.h>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <fstream>
#include "LuaGenerator.hpp"
#include "FindPath.hpp"

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


    const string wcFile = "../../Workcell3/WC3_Scene.wc.xml";
    const string deviceName = "UR1";

    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);
    Q to(6,1.701,-0.081,0.664,3.358,-0.125,-3.314);

    FindPath Mypath(wcFile, deviceName);
    Mypath. moveObstacle(0, -0.175, 1);
    QPath path = Mypath.getPath(to,from,0.2,10.);
    //Mypath.printPath();

    // Write path to LUA script
    LuaGenerator lua;
    lua.generateLua(path, "luascript.txt");

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
            Qstring.append(",");



        string_state_vec.push_back(Qstring);
    }

    for (int i = 0; i < string_state_vec.size() ; i++){
        mySocket.create_client(112);
        mySocket.sendM(string_state_vec[i]);
        usleep(20000);
    }



    cout << "Program done." << endl;
    return 0;
}
