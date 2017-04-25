#include <iostream>
#include <fstream>
#include <rw/rw.hpp>
#include "findPathWithEpsilon.hpp"
#include "LuaGenerator.hpp"
#include "Utilities.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
using namespace std;
using namespace rw::math;

int main(int argc, char** argv)
{
	Q from(6,-3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
	Q to(6,1.571, 0.006, 0.03, 0.153, 0.762, 4.49);

	findPathWithEpsilon test(argv[1], "KukaKr16");
    double epsilon = 0;
    for(epsilon = 0.005; epsilon < 2 * M_PI; epsilon += 0.005)
    {
        for(uint8_t i = 0; i < 200; i++)
        {
            double time = 0;
        	QPath path = test.findPath(epsilon,from,to, time);
            std::cout << epsilon << "," <<  get_path_length(path) << "," << time << std::endl;
            if(argc > 2 and std::string(argv[2]) == std::string("-getlua"))
            {
                LuaGenerator lua;
                lua.make_preample();
                lua.add_start_point(from);
                for(auto &q : path) lua.add_point(q);
                lua.add_end();
                std::cout << lua.get_string();
            }
        }
    }

    return 0;
}
