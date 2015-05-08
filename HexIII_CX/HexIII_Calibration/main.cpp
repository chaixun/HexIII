#include <iostream>
#include <Aris_ControlData.h>
#include <Aris_Control.h>
#include <Aris_Message.h>
#include <Aris_Socket.h>
#include <Aris_Core.h>

#include "Control.h"
#include "Server.h"

//#include "kinect.h"

using namespace std;

Aris::RT_CONTROL::ACTUATION cs;
Aris::RT_CONTROL::CSysInitParameters initParam;

static int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
{
    -15849882,	 -16354509,	 -16354509,
    -15849882,	 -16354509,	 -16354509,
    -15849882,	 -16354509,	 -16354509,
    -16354509,	 -15849882,	 -16354509,
    -15849882,	 -16354509,	 -16354509,
    -16354509,	 -16354509,  -15849882
};

//Kinect visionsensor;
int num = 1;

int main()
{
  //  visionsensor.viewcloud();
  // visionsensor.start();

    //msg call back
    Aris::Core::RegisterMsgCallback(CS_Connected,On_CS_Connected);
    Aris::Core::RegisterMsgCallback(CS_CMD_Received,On_CS_CMD_Received);
    Aris::Core::RegisterMsgCallback(CS_Lost,On_CS_Lost);

    Aris::Core::RegisterMsgCallback(VS_Connected,On_VS_Connected);
    Aris::Core::RegisterMsgCallback(VS_Capture,On_VS_Capture);
    Aris::Core::RegisterMsgCallback(VS_Lost,On_VS_Lost);


    Aris::Core::RegisterMsgCallback(RC_HOME, autoHome);
    Aris::Core::RegisterMsgCallback(RC_Enable, autoEnable);
    Aris::Core::RegisterMsgCallback(RC_Start, autoStart);
    Aris::Core::RegisterMsgCallback(RC_STOP, autoStop);

    Aris::Core::RegisterMsgCallback(RC_MOVE, autoMove);
    Aris::Core::RegisterMsgCallback(RC_MOVE_BACK, autoMoveBack);
    Aris::Core::RegisterMsgCallback(RC_AUTO_CALIBRATION, autoCalibration);
    Aris::Core::RegisterMsgCallback(RC_TURN_LEFT,autoTurnLeft);
    Aris::Core::RegisterMsgCallback(RC_TURN_RIGHT,autoTurnRight);

    //CONN call back
    ControlSystem.SetCallBackOnReceivedConnection(On_CS_ConnectionReceived);
    ControlSystem.SetCallBackOnReceivedData(On_CS_DataReceived);
    ControlSystem.SetCallBackOnLoseConnection(On_CS_ConnectionLost);
    ControlSystem.StartServer("5690");

    VisionSystem.SetCallBackOnReceivedConnection(On_VS_ConnectionReceived);
    VisionSystem.SetCallBackOnReceivedData(On_VS_DataReceived);
    VisionSystem.SetCallBackOnLoseConnection(On_VS_ConnectionLost);
    VisionSystem.StartServer("5691");

    Aris::Core::THREAD T1;
    T1.SetFunction(Thread1);
    T1.Start(0);
    cs.SetSysInitializer(initFun);

    cs.SetOnHomed(autoH2R);
    cs.SetTrajectoryGenerator(tg);
    cs.SetModeCycVel();

    initParam.homeMode=17;
    initParam.homeOffset=HEXBOT_HOME_OFFSETS_RESOLVER;
    initParam.startMotorID=0;
    //initParam.endMotorID=1;
    initParam.endMotorID=18;
    initParam.motorNum=18;
    initParam.homeHighSpeed=200000;
    initParam.homeLowSpeed=15000;
    cs.SysInit(initParam);
    cs.SysInitCommunication();
    cs.SysStart();

    cout<<"Will start"<<endl;

    while(!cs.IsSysStopped())
    {
        sleep(1);
    }

    return 0;
}


