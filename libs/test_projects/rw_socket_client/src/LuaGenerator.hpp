#pragma once
#include <string>
#include <rw/rw.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::trajectory;


class LuaGenerator
{
    public:
        void generateLua(QPath &path, std::string filepath);
};
