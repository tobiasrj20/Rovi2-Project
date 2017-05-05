#include "SamplePlugin.hpp"
#include <RobWorkStudio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include "SocketCommunication.hpp"

using namespace std;
using rw::kinematics::State;
using rw::models::WorkCell;
using rws::RobWorkStudioPlugin;

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
}

SamplePlugin::~SamplePlugin()
{
}

void SamplePlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
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
        socket_server();
    }
    else {
        _btn0->setText("Connect");
        _label0->setText("Status: Not connected to ROS.");
    }
}

void SamplePlugin::stateChangedListener(const State& state) {

}


void SamplePlugin::moveRobot() {
    /*
    rw::models::Device::Ptr device;

    Q qVector = device->getQ(state);
    device = _wc->findDevice("PA10");
    device->setQ(qVector, state);
    getRobWorkStudio()->setState(state);*/
}

void SamplePlugin::moveObstacle() {
    /*rw::models::WorkCell::Ptr _wc;
    // Auto load workcell
	WorkCell::Ptr _wc = WorkCellLoader::Factory::load("/home/student/Downloads/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(_wc);

    state = _wc->getDefaultState();
    markerMotions = readMotionFile("/home/student/Downloads/SamplePluginPA10/motions/MarkerMotion" SPEED ".txt");
    MovableFrame* marker = (MovableFrame*) _wc->findFrame("Marker");
    marker->moveTo(markerMotions[0], state);
    getRobWorkStudio()->setState(state);*/
}

void SamplePlugin::socket_server() {
    SocketCommunication Socket;
    Socket.connect(112);
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SamplePlugin);
#endif
