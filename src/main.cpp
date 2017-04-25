#include <iostream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <fstream>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 10.

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
}

void outputLUA(QPath &path){
        ofstream myfile;
        myfile.open ("/home/age/LAGER/Dropbox/SDU/Gruppearbejde/RoVi2_final/LUA/LUA_man_ex_2.txt");
        myfile << "wc = rws.getRobWorkStudio():getWorkCell()\n";
        myfile << "state = wc:getDefaultState()\n";
        myfile << "device = wc:findDevice(\"UR1\")\n";
        myfile << "\n\n";
        myfile << "function setQ(q)\n";
        myfile << "    qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n";
        myfile << "    device:setQ(qq,state)\n";
        myfile << "    rws.getRobWorkStudio():setState(state)\n";
        myfile << "    rw.sleep(0.1)\n";
        myfile << "end\n\n";

        for (QPath::iterator it = path.begin(); it < path.end(); it++) {
            myfile << "setQ({"<< (*it)[0] << "," << (*it)[1] << "," << (*it)[2] << "," << (*it)[3] << "," << (*it)[4] << "," << (*it)[5] << "})\n";
        }
        myfile.close();
}

int main(int argc, char** argv) {

    rw::math::Math::seed();

    const string wcFile = "/home/age/LAGER/Dropbox/SDU/Gruppearbejde/RoVi2_final/Workcell3/WC3_Scene.wc.xml";
    const string deviceName = "UR1";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;


    // Load workcell and device and throw error if no device
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }

    State state = wc->getDefaultState();




    Q from(6,-0.702,-2.528,-0.573,5.927,1.72,-1.253);

    device->setQ(from,state);
    //Kinematics::gripFrame(bottle,tool,state);

    Q to(6,1.701,-0.081,0.664,3.358,-0.125,-3.114);

    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

    /** Most easy way: uses default parameters based on given device
        sampler: QSampler::makeUniform(device)
        metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
        extend: 0.05 */
    //QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);

    /** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    double extend = 0.00;
    QToQPlanner::Ptr planner;
    QPath path;
    Timer t;



    if (!checkCollisions(device, state, detector, from))
        return 0;
    if (!checkCollisions(device, state, detector, to))
        return 0;

    if(0){
        // Computing 1800 paths using different values for epsilon and save to file
        for(int i = 0 ; i < 180 ; i++){
            extend += 0.0334;
            for(int j = 0 ; j < 10 ; j++){
                path.clear();
                planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
                t.resetAndResume();
                planner->query(from,to,path,MAXTIME);
                t.pause();
                double dist = 0;

                for (int k = 0 ; k < path.size()-1 ; k++){
                    dist += sqrt(pow(path[k][0]-path[k+1][0],2)+
                                 pow(path[k][1]-path[k+1][1],2)+
                                 pow(path[k][2]-path[k+1][2],2)+
                                 pow(path[k][3]-path[k+1][3],2)+
                                 pow(path[k][4]-path[k+1][4],2)+
                                 pow(path[k][5]-path[k+1][5],2));
                }
            }
        }
    }

    // Single path generation
    extend = 0.6;
    path.clear();
    planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);
    t.resetAndResume();
    planner->query(from,to,path,MAXTIME);
    t.pause();


    if (t.getTime() >= MAXTIME) {
        cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
    }

    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        cout << *it << endl;
    }

    // Write path to LUA script
    outputLUA(path);

    cout << "Program done." << endl;
    return 0;
}
