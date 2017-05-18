#pragma once
#include <rw/rw.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::pathplanning;
using namespace rw::trajectory;

class EdgeCollisionDetectors
{
    public:
        EdgeCollisionDetectors(const PlannerConstraint &constraint, double epsilon);
        bool inCollisionBinary(Q to, Q from);

        // TEST SECTION!
        void inCollisionBinaryTest();

    private:
        PlannerConstraint _constraint;
        double _epsilon;
        bool inCollision(const Q& q);
};
