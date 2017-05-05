#include "SamplePlugin.hpp"
#include <RobWorkStudio.hpp>
#include <boost/bind.hpp>
#include <iostream>


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

SamplePlugin::~SamplePlugin()
{
}

/*
*   OBS. this function is called automatically when the plugin is loaded into RobworkStudio
*/
void SamplePlugin::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
;
    _wc = WorkCellLoader::Factory::load("/home/tobias/Dropbox/RobTek/Cand_2_semester/Rovi2-Project/Workcell3/WC3_Scene.wc.xml");
    //_wc = WorkCellLoader::Factory::load("../../Workcell3/WC3_Scene.wc.xml");
	_state = _wc->getDefaultState();        // Get workcell state
    getRobWorkStudio()->setWorkCell(_wc);   // Set workcell in RobworkStudio
    getRobWorkStudio()->setState(_state);   // Set state in RobworkStudio
}

void SamplePlugin::open(WorkCell* workcell)
{
}

void SamplePlugin::close() {
}

void SamplePlugin::btnPressed() {
    if(_label0->text() == "Status: Not connected to ROS.") {
        _label0->setText("Status: Connected to ROS.");
        _btn0->setText("Disconnect");
        _timer->start(100); // Run FPS 10 Hz
        //Socket.createServer();  // Create server for communication between ROS and RobworkStudio

        //moveRobot(Q(6, 67.492, -89.954, 0.000, -89.954, 0.000, 0.000));
        moveObstacle(-1.880, 0.000, 1.717);
    }
    else {
        _btn0->setText("Connect");
        _label0->setText("Status: Not connected to ROS.");
        _timer->stop();
        //Socket.stopServer();
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

void SamplePlugin::timer()
{
    /*
    if(Socket.dataReady())
    {
        state = Socket.getState();
        moveRobot();
        moveObstacle();
    }*/
}


#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SamplePlugin);
#endif
