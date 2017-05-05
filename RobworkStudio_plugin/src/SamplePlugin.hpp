#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include <rws/RobWorkStudioPlugin.hpp>

#include "ui_SamplePlugin.h"

#include <rws/RobWorkStudio.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <QTimer>

#include "SocketCommunication.hpp"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using namespace rw::math;
using namespace rws;

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
    Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
    #if RWS_USE_QT5
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
    #endif

    public:
        SamplePlugin();
    	virtual ~SamplePlugin();

        virtual void open(rw::models::WorkCell* workcell);

        virtual void close();

        virtual void initialize();
        void moveRobot(Q q);
        void moveObstacle(double x, double y, double z);

    private:
        rw::models::WorkCell::Ptr _wc;  // Workcell
        rw::kinematics::State _state;   // Workcell state
        rw::models::Device::Ptr _device; // Robot device
        QTimer* _timer; // FPS timer
        SocketCommunication Socket;     // Communication between ROS and RobworkStudio


    private slots:
        void btnPressed();  // Connect/disconnect to socket and start/stop timer
        void timer();   // Fetch data from socket and move robot and obstacle
        void stateChangedListener(const rw::kinematics::State& state);
};

#endif
