#pragma once
#include <string>
#include <rw/rw.hpp>
using namespace rw::math;

class LuaGenerator
{
    public:
    std::string lua_string = "";
    void make_preample();

    void add_start_point(const Q &q);
    void add_point(const Q &q);
    std::string get_setQ(const Q &q);
    std::string get_string()
        { return lua_string; }
    void add_end();
};
