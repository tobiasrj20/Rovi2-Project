#include "RRT.hpp"
#include <boost/foreach.hpp>
#include <iostream>

using namespace std;

RRT::RRT(const PlannerConstraint& constraint, QSampler::Ptr sampler, QMetric::Ptr metric)
:_edgeCollisionDetect(constraint, 0.1)
{
    _constraint = constraint;
    _sampler = sampler;
    _metric = metric;
}

bool RRT::inCollision(const Q& q)
{
    return _constraint.getQConstraint().inCollision(q);
}

// Local planner: 'a' is known to be collision free, but 'b' is not.
bool RRT::inCollision(const Q &a, const Q &b)
{
    //return _constraint.getQConstraint().inCollision(b) || _constraint.getQEdgeConstraint().inCollision(a, b); // Robwork edge collision checker
    return _constraint.getQConstraint().inCollision(b) || _edgeCollisionDetect.inCollisionBinary(a, b);   // Own edge collision checker
}

/*
*   RRT extend
*   Tries to extend the tree a epsilon step towards 'q'
*/
ExtendResult RRT::extend(Tree& tree, const Q& q, Node *nearestNode, const double &epsilon)
{
    Q qNear = nearestNode->getValue();

    //  NEW_CONFIG: Create qNew
    const Q delta = q - qNear;
    const double dist = _metric->distance(delta);
    const Q qNew = qNear + epsilon*(delta/dist);    // delta/dist = a unit vector from qNear to q

    //  Check edge beween qNear and qNew for collisions
    if(dist <= epsilon)
    {
        if(!inCollision(qNear, q))
        {
            tree.add(q, nearestNode);
            return REACHED;
        }
    }
    else if(!inCollision(qNear, qNew))
    {
        tree.add(qNew, nearestNode);

        if(qNew == q)
            return REACHED;
        else
            return ADVANCED;
    }
    return TRAPPED;
}

/*
*   Loops through the tree and find the node nearest to q
*   return a pointer to the nearest tree node
*/
Node* RRT::nearestNeighbor(Tree &tree, const Q &q)
{
    double minLength = DBL_MAX;  // Set to max size of a double
    Node* nearestNode = NULL;

    BOOST_FOREACH(Node* node, tree.getNodes())  // Make own tree and another loop?
    {
        double length = _metric->distance(q, node->getValue()); // length between q and the node in the tree
        if(length < minLength)
        {
            minLength = length;
            nearestNode = node;
        }

    }

    return nearestNode;
}

ExtendResult RRT::connect(Tree &tree, const Q &q, double epsilon)
{
    ExtendResult s = ADVANCED;
    Node* nearestNode = nearestNeighbor(tree, q);
    Q qNear = nearestNode->getValue();

    while(s == ADVANCED)
    {
        s = extend(tree, q, nearestNode, epsilon);

        if(s == ADVANCED)
        {
            nearestNode = &tree.getLast();
        }

    }
    return s;
}

QPath RRT::rrtConnectPlanner(Q qInit, Q qGoal, double epsilon, double maxTime)
{
    timer t;
    Tree startTree(qInit);
    Tree goalTree(qGoal);
    Tree* treeA = &startTree;
    Tree* treeB = &goalTree;

    // Initial collision checks
    if (inCollision(qInit))
    {
        cout << "RRT: qInit is in collision!" << endl;
        return QPath();
    }

    if (inCollision(qGoal))
    {
        cout << "RRT: qGoal is in collision!" << endl;
        return QPath();
    }


    if (!_constraint.getQEdgeConstraint().inCollision(qInit, qGoal))
    {
        QPath result;
        result.push_back(qInit);
        result.push_back(qGoal);
        return result;
    }

    while(t.millisec_elapsed() < maxTime)
    {
        // Get random sample
        const Q qRand = _sampler->sample();
        if(qRand.empty())
            cout << "ERROR: RRT could not make a random sample!" << endl;

        // Find nearest neighbor
        Node* nearestNode = nearestNeighbor(*treeA, qRand);
        Q qNear = nearestNode->getValue();

        if(!extend(*treeA, qRand, nearestNode, epsilon) == TRAPPED)
        {
            Node *qNewNode = &treeA->getLast();
            Q qNew = qNewNode->getValue();

            if(connect(*treeB, qNew, epsilon) == REACHED)
            {
                return getPath(startTree, goalTree);    // Success, return the path rrt found
            }
        }
        std::swap(treeA, treeB);
    }

    return QPath(); // Error: Return empty QPath
}

QPath RRT::getPath(const Tree &startTree, const Tree &goalTree)
{
    QPath revPart;
    QPath result;

    Tree::getRootPath(*startTree.getLast().getParent(), revPart);
    result.insert(result.end(), revPart.rbegin(), revPart.rend());
    Tree::getRootPath(goalTree.getLast(), result);

    return result;
}


/*
*       TEST section !!!!!!!!!!!!!!!!!!!
*/


void RRT::connectTest()
{
    Q q(6, 1.0, 0.0, -1.0, -1.0, 0.0, 0.0);
    Q qNear(6, 1.0, -3.3, -1.0, -1.0, 0.0, 0.0);    // Use to test ADVANCED


    Tree tree(qNear);
    Node *nearNode = &tree.getLast();

    ExtendResult res = connect(tree, q, 0.5);

    if(res == TRAPPED)
        cout << "TRAPPED" << endl;
    if(res == REACHED)
        cout << "REACHED" << endl;
    if(res == ADVANCED)
        cout << "ADVANCED" << endl;

    nearNode = &tree.getLast();
    Q qNew = nearNode->getValue();
    cout << "QNew: " << qNew << endl;
}

void RRT::extendTest()
{
    Q q(6, 1.0, 0.0, -1.0, -1.0, 0.0, 0.0);
    //Q qNear(6, 1.0, -0.8, -1.0, -1.0, 0.0, 0.0);    // Use to test ADVANCED
    Q qNear(6, 1.0, -0.5, -1.0, -1.0, 0.0, 0.0);    // Use to test REACHED

    Tree tree(qNear);
    Node *nearNode = &tree.getLast();

    ExtendResult res = extend(tree, q, nearNode, 0.5);

    if(res == TRAPPED)
        cout << "TRAPPED" << endl;
    if(res == REACHED)
        cout << "REACHED" << endl;
    if(res == ADVANCED)
        cout << "ADVANCED" << endl;

    nearNode = &tree.getLast();
    Q qNew = nearNode->getValue();
    cout << "QNew: " << qNew << endl;
}

void RRT::nearestNeighborTest()
{
    Q q1(6, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0);
    Q q2(6, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0);
    Q q3(6, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0);

    Q q4(6, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0);

    Tree tree(q1);
    Node *node = &tree.getLast();
    tree.add(q2,node);
    node = &tree.getLast();
    tree.add(q2,node);

    Node* qNearNode = nearestNeighbor(tree, q4);

    cout << qNearNode->getValue() << endl;
}
