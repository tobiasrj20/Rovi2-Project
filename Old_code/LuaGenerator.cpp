#include "LuaGenerator.hpp"
#include <sstream>
void LuaGenerator::make_preample()
{
    std::stringstream s;
    s << "wc = rws.getRobWorkStudio():getWorkCell()\n";
    s << "state = wc:getDefaultState()\n";
    s << "device = wc:findDevice(\"KukaKr16\")\n";
    s << "gripper = wc:findFrame(\"Tool\");\n";
    s << "bottle = wc:findFrame(\"Bottle\");\n";
    s << "table = wc:findFrame(\"Table\");\n";

    s << "function setQ(q)\n";
    s << "qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n";
    s << "device:setQ(qq,state)\n";
    s << "rws.getRobWorkStudio():setState(state)\n";
    s << "rw.sleep(0.1)\n";
    s << "end\n";

    s << "function attach(obj, tool)\n";
    s << "rw.gripFrame(obj, tool, state)\n";
    s << "rws.getRobWorkStudio():setState(state)\n";
    s << "rw.sleep(0.1)\n";
    s << "end\n";
    lua_string += s.str();
}

void LuaGenerator::add_start_point(const Q &q)
{
    std::stringstream s;
    s << get_setQ(q);
    s << "attach(bottle,gripper)\n";
    lua_string += s.str();
}

std::string LuaGenerator::get_setQ(const Q &q)
{
    std::stringstream s;
    s << "setQ({";
    s << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", "
    << q[4] << ", " << q[5] << "})\n";
    return s.str();
}
void LuaGenerator::add_point(const Q &q)
{
    std::stringstream s;
    s << get_setQ(q);
    lua_string += s.str();
}
void LuaGenerator::add_end()
{
    lua_string += "attach(bottle,table)\n";
}
