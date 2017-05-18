#include "EdgeCollisionDetectors.hpp"
#include <iostream>

using namespace std;

EdgeCollisionDetectors::EdgeCollisionDetectors(const PlannerConstraint &constraint, double epsilon)
{
    _constraint = constraint;
    _epsilon = epsilon;
}

bool EdgeCollisionDetectors::inCollisionBinary(Q to, Q from)
{
    Q q;
    Q deltaQ = to-from;

    int n = deltaQ.norm2()/_epsilon;
    int levels = ceil(log2(n));

    for(int i = 1; i <= levels; i++){
        int steps = pow(2.0,i-1);
        Q step = deltaQ/steps;
        for(int j = 1; j <= steps; j++){

            q = from + (j - 0.5)*step;

            if(inCollision(q))
            {
                return true;
            }
        }
    }
    return false;
}

bool EdgeCollisionDetectors::inCollision(const Q& q)
{
    return _constraint.getQConstraint().inCollision(q);
}


/*
*   TEST SECTION!!!!
*/

void EdgeCollisionDetectors::inCollisionBinaryTest()
{
    Q from(6, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0);
    Q to(6, 0.8, -0.5, 0.0, -1.0, 0.0, 0.0);    // Use to test REACHED

    cout << "Local planner should succeed = " << inCollisionBinary(from, to) << endl;

    Q to2(6, 0.0, 0.2, 0.0, -1.0, 0.0, 0.0);    // Use to test TRAPPED

    cout << "Local planner should fail = " << inCollisionBinary(from, to2) << endl;
}
