#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include <rws/RobWorkStudioPlugin.hpp>

#include "ui_SamplePlugin.h"

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
        void socket_server();
        void error();
        void moveRobot();
        void moveObstacle();

    private:
        //rw::models::WorkCell::Ptr _wc;  // Workcell pointer

    private slots:
        void btnPressed();

        void stateChangedListener(const rw::kinematics::State& state);
};

#endif
