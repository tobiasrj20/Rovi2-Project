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
    }
    else {
        _btn0->setText("Connect");
        _label0->setText("Status: Not connected to ROS.");
    }
}

void SamplePlugin::stateChangedListener(const State& state) {

}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(SamplePlugin);
#endif
