#include <iostream>
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


    const string wcFile = "../Workcell3/WC3_Scene.wc.xml";
    const string deviceName = "UR1";

    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);
    Q to(6,1.701,-0.081,0.664,3.358,-0.125,-3.314);

    FindPath Mypath(wcFile, deviceName);
    QPath path = Mypath.getPath(to,from,0.6,10.);
    Mypath.printPath();

    // Write path to LUA script
    LuaGenerator lua;
    lua.generateLua(path, "luascript.txt");

    cout << "Program done." << endl;
    return 0;
}
