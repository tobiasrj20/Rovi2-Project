#include "LuaGenerator.hpp"

void LuaGenerator::generateLua(QPath &path, std::string filepath)
{
    ofstream myfile;
    myfile.open(filepath);

    myfile << "wc = rws.getRobWorkStudio():getWorkCell()\n";
    myfile << "state = wc:getDefaultState()\n";
    myfile << "device = wc:findDevice(\"UR1\")\n";
    myfile << "obstacle = wc:findFrame(\"Obstacle\")\n";
    myfile << "\n\n";
    myfile << "function setQ(t)\n";
    myfile << "    qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n";
    myfile << "    device:setQ(qq,state)\n";
    myfile << "    rws.getRobWorkStudio():setState(state)\n";
    myfile << "    rw.sleep(0.1)\n";
    myfile << "end\n\n";
/*  myfile << "function moveObstacle(q)\n";
    myfile << "    obstacle.moveTo(,state)\n";
    myfile << "    rws.getRobWorkStudio():setState(state)\n";
    myfile << "    rw.sleep(0.1)\n";
    myfile << "end\n\n"; */
    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        myfile << "setQ({"<< (*it)[0] << "," << (*it)[1] << "," << (*it)[2] << "," << (*it)[3] << "," << (*it)[4] << "," << (*it)[5] << "})\n";
    }
    myfile.close();

}
