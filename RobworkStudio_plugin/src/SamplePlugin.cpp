#include "SamplePlugin.hpp"
#include <RobWorkStudio.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include <vector>

using namespace std;
using rw::kinematics::State;
using rw::models::WorkCell;
using rws::RobWorkStudioPlugin;



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

}

SamplePlugin::~SamplePlugin() {
}

void SamplePlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

    //_wc = WorkCellLoader::Factory::load("/home/age/Desktop/SDU/8_semester/RoVi2/Final/workspace/Rovi2-Project/Workcell3/WC3_Scene.wc.xml");

    _wc = WorkCellLoader::Factory::load("/home/tobias/Dropbox/RobTek/Cand_2_semester/Rovi2-Project/Workcell3/WC3_Scene.wc.xml");
    //_wc = WorkCellLoader::Factory::load("../../Workcell3/WC3_Scene.wc.xml");
    _state = _wc->getDefaultState();        // Get workcell state
    getRobWorkStudio()->setWorkCell(_wc);   // Set workcell in RobworkStudio
    getRobWorkStudio()->setState(_state);   // Set state in RobworkStudio
}

void SamplePlugin::timer() {

    if(mySocket.dataReady()){
        std::cout << "Q accepted" << std::endl;
        string buf;
        state_vec.clear();
        mySocket.receive(buf);
        std::string token;
        std::string::size_type sz;     // alias of size_t
        std::istringstream ss(buf);

        while(std::getline(ss, token, ',')) {
            state_vec.push_back( std::stof (token,&sz));
        }

        std::cout << "SetQ(6,";
        for (uint i = 0; i < 6; i++) {
            std::cout << state_vec[i] << ",";
        }
        std::cout << ")" << std::endl;

        std::cout << "SetObstacle(";
        for (uint i = 6; i < 9; i++) {
            std::cout << state_vec[i] << ",";
        }
        std::cout << ")" << std::endl;

        moveRobot(Q(6, state_vec[0],state_vec[1],state_vec[2],state_vec[3],state_vec[4],state_vec[5]));
        moveObstacle(state_vec[6],state_vec[7],state_vec[8]);
    }
}

void SamplePlugin::open(WorkCell* workcell) {
}

void SamplePlugin::close() {
}

void SamplePlugin::btnPressed() {
    if(_label0->text() == "Status: Not connected to ROS.") {
        _label0->setText("Status: Connected to ROS.");
        _btn0->setText("Disconnect");
        _timer->start(10); // Run FPS 200 Hz
        mySocket.runServerThread();
    }
    else {
        _btn0->setText("Connect");
        _label0->setText("Status: Not connected to ROS.");
        _timer->stop();
    }
}

void SamplePlugin::stateChangedListener(const State& state) {
}

void SamplePlugin::moveRobot(Q q)
{
    _device = _wc->findDevice("UR1");
    _device->setQ(q, _state);
    getRobWorkStudio()->setState(_state);
    //Q qVector = device->getQ(state);
}

void SamplePlugin::moveObstacle(double x, double y, double z)
{
    double roll = 0.000;
    double pitch = 0.000;
    double yaw = 0.000;

    RPY<double> rpy(roll,pitch,yaw);   // Create RPY matrix
    Vector3D<double> xyz(x,y,z);   // Create translation vector
    Transform3D<double> t_matrix(xyz, rpy.toRotation3D() ); // Create a transformation matrix from the RPY and XYZ

    MovableFrame* obstacle = (MovableFrame*) _wc->findFrame("Obstacle");
    obstacle->moveTo(t_matrix, _state);
    getRobWorkStudio()->setState(_state);
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SamplePlugin);
#endif
