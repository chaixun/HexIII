#include "Control.h"

void* Thread1(void *)
{
    Aris::Core::RunMsgLoop();
    return NULL;
}

int initFun(Aris::RT_CONTROL::CSysInitParameters& param)
{
    gait.InitGait(param);
    return 0;
}

int tg(Aris::RT_CONTROL::CMachineData& data)
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        data.commandData[i].Position=data.feedbackData[i].Position;
    }

    if(cs.RT_IsCusMsg())
    {
        cs.DataRecv->Paste(&gaitcmdtemp,sizeof(gaitcmdtemp));
        rt_printf("RT_GAIT get %d\n",gaitcmdtemp);
        if(!gait.IsGaitFinished())
        {
            if(gaitcmdtemp!=GAIT_MOVE&&gaitcmdtemp!=GAIT_MOVE_BACK)
            {
                rt_printf("%d ommited\n",gaitcmdtemp);
            }
            else
            {
                gait.Gait_iter=gait.Gait_iter+1;
                rt_printf("d% times gait moving",gait.Gait_iter);
            }
        }
        else
        {
            gaitcmd=gaitcmdtemp;
            gait.Gait_iter=1;
        }

    }
    gait.RunGait(gaitcmd,data);
    if(gait.IsGaitFinished())
    {
        if (CGait::IsAutoCalibrateRegistered == true && CGait::IsAutoCalibrateStepRegistered == true)
        {
            rt_printf("AUTO CALIBRATION STEP FINISHED!!!!!\n");
            CGait::IsAutoCalibrateStepRegistered = false;
            Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
        }
    }
    return 0;
}

int autoH2R(void * arg)
{   
    cout<<"autoH2E called"<<endl;
    cs.NRT_MCHomeToRunning();
    return 0;
}

int autoPowerOff(void*)
{   
    cout<<"auto poweroff send"<<endl;
    cs.NRT_MCPowerOff();
    return 0;
}

int autoEnable(Aris::Core::MSG &msg)
{
    cs.NRT_MCEnable();
    return 0;
}

int autoStop(Aris::Core::MSG &msg)
{
    cs.NRT_MCStop();
    return 0;
}

int autoHome(Aris::Core::MSG &msg)
{
    cs.NRT_MCHome();
    return 0;
}

int autoStart(Aris::Core::MSG &msg)
{

    EGAIT cmd=GAIT_HOME2START;
    Aris::Core::MSG data;

    data.SetLength(sizeof(cmd));
    data.Copy(&cmd,sizeof(cmd));
    cs.NRT_SendData(data);

    return 0;
}

int autoMove(Aris::Core::MSG &msg)
{

    EGAIT cmd=GAIT_MOVE;

    Aris::Core::MSG data;
    data.SetLength(sizeof(cmd));
    data.Copy(&cmd,sizeof(cmd));
    cs.NRT_SendData(data);
    return 0;
}

int autoMoveBack(Aris::Core::MSG &msg)
{
    EGAIT cmd=GAIT_MOVE_BACK;
    Aris::Core::MSG data;
    data.SetLength(sizeof(cmd));
    data.Copy(&cmd,sizeof(cmd));
    cs.NRT_SendData(data);
    return 0;
}


int autoCalibration(Aris::Core::MSG &msg)
{
    Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
    CGait::IsAutoCalibrateRegistered=true;
    return 0;
}

int autoTurnLeft(Aris::Core::MSG &msg)
{
    EGAIT cmd=GAIT_TURN_LEFT;
    Aris::Core::MSG data;
    data.SetLength(sizeof(cmd));
    data.Copy(&cmd,sizeof(cmd));
    cs.NRT_SendData(data);
    return 0;
}

int autoTurnRight(Aris::Core::MSG &msg)
{
    EGAIT cmd=GAIT_TURN_RIGHT;
    Aris::Core::MSG data;
    data.SetLength(sizeof(cmd));
    data.Copy(&cmd,sizeof(cmd));
    cs.NRT_SendData(data);
    return 0;
}



