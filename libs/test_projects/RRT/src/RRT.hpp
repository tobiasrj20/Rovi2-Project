#pragma once

// std libs
#include <utility>
#include <chrono>

// Robwork libs
#include <rw/rw.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>

#include "RRTTree.hpp"
#include "RRTNode.hpp"

// Own libs
#include "EdgeCollisionDetectors.hpp"

using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::pathplanning;
using namespace rw::trajectory;

// Type definitions, maybe this should be removed
typedef rwlibs::pathplanners::RRTNode<rw::math::Q> Node;
typedef rwlibs::pathplanners::RRTTree<rw::math::Q> Tree;

// Enumarations
enum ExtendResult { TRAPPED, REACHED, ADVANCED };

class RRT
{
    public:
        RRT(const PlannerConstraint& constraint, QSampler::Ptr sampler, QMetric::Ptr metric, double epsilonEdgeColDetect);
        QPath rrtConnectPlanner(Q qInit, Q qGoal, double epsilon, double maxTime);

        // TEST SECTION!!!
        void connectTest();
        void nearestNeighborTest();
        void extendTest();

    private:
        PlannerConstraint _constraint;
        QSampler::Ptr _sampler;
		QMetric::Ptr _metric;
        EdgeCollisionDetectors _edgeCollisionDetect;

        ExtendResult connect(Tree &tree, const Q &q, double epsilon);
        ExtendResult extend(Tree& tree, const Q& q, Node *nearestNode, const double &epsilon);
        Node* nearestNeighbor(Tree& tree, const Q& q);
        bool inCollision(const Q& q);
        bool inCollision(const Q &a, const Q &b);
        QPath getPath(const Tree &startTree, const Tree &goalTree);
};

struct timer
{
    typedef std::chrono::steady_clock clock ;
    typedef std::chrono::seconds seconds ;

    public:
        void reset()
        {
            start = clock::now();
        }

        unsigned long long millisec_elapsed() const
        {
            return std::chrono::duration_cast<seconds>( clock::now() - start ).count();
        }

    private:
        clock::time_point start = clock::now() ;
};
