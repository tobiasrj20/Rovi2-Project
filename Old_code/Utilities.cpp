#include "Utilities.hpp"
#include <cstdint>
#include <cmath>
double get_path_length(QPath &path)
{
    double length = 0;
    for(size_t i = 1; i < path.size(); i++)
    {   double tmp = get_dist(path[i - 1], path[i]);
        length += tmp;
    }
    return length;
}

double get_dist(Q &q1, Q &q2)
{
    auto diff = q1 - q2;
    double dist = 0;
    for(uint8_t i = 0; i < 6; i++)
    {
        dist += pow(diff[i], 2);
    }
    return sqrt(dist);
}
