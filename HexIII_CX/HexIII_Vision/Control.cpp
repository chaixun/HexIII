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
        cs.DataRecv->Paste(&gaitcmd,sizeof(gaitcmd));
        rt_printf("RT_GAIT get %d\n",gaitcmd);
    }
    gait.RunGait(gaitcmd,data);
    if(gait.IsGaitFinished())
    {
        if (CGait::IsWalkAdaptiveRegistered == true && CGait::IsWalkAdaptiveStepRegistered == true)
        {
            rt_printf("WALK ADAPTIVE STEP FINISHED!!!!!\n");
            CGait::IsWalkAdaptiveStepRegistered = false;
            Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
        }

        if (CGait::IsWalkAvoidRegistered == true && CGait::IsWalkAvoidStepRegistered == true)
        {
            rt_printf("WALK AVOID STEP FINISHED!!!!!\n");
            CGait::IsWalkAvoidStepRegistered = false;
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


int autoAdaptive(Aris::Core::MSG &msg)
{
    CGait::IsWalkAdaptiveRegistered=true;
    Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
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

int autoBeginDiscover(Aris::Core::MSG &msg)
{
    EGAIT cmd=GAIT_BEGIN_DISCOVER;
    Aris::Core::MSG data;
    data.SetLength(sizeof(cmd));
    data.Copy(&cmd,sizeof(cmd));
    cs.NRT_SendData(data);
    return 0;
}

int autoEndDiscover(Aris::Core::MSG &msg)
{
    EGAIT cmd=GAIT_END_DISCOVER;
    Aris::Core::MSG data;
    data.SetLength(sizeof(cmd));
    data.Copy(&cmd,sizeof(cmd));
    cs.NRT_SendData(data);
    return 0;
}

int autoAvoid(Aris::Core::MSG &msg)
{
    CGait::IsWalkAvoidRegistered=true;
    Aris::Core::PostMsg(Aris::Core::MSG(VS_Capture));
    return 0;
}



