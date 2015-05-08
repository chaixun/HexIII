#include "Gait.h"

bool CGait::IsConsFinished;
bool CGait::IsAdaptiveGaitRegistered;
bool CGait::IsAutoCalibrateRegistered;
bool CGait::IsAutoCalibrateStepRegistered;
int CGait::Gait_iter;
int CGait::Gait_iter_count;

int CGait::GaitAutoCalib[GAIT_AUTOCALIB_LEN][GAIT_WIDTH];

EGAIT CGait::m_currentGait;
long long int CGait::m_gaitStartTime;
int CGait::m_gaitCurrentIndex;
EGaitState CGait::m_gaitState;
Aris::RT_CONTROL::CMotorData CGait::m_standStillData[MOTOR_NUM];
Aris::RT_CONTROL::CMotorData CGait::m_commandDataMapped[MOTOR_NUM];
Aris::RT_CONTROL::CMotorData CGait::m_feedbackDataMapped[MOTOR_NUM];

int GaitHome2Start[GAIT_HOME2START_LEN][GAIT_WIDTH];

///////////////////////////////////////////////
int GaitMove_Acc[GAIT_ACC_LEN][GAIT_WIDTH];
int GaitMove_Cons[GAIT_CON_LEN][GAIT_WIDTH];
int GaitMove_Dec[GAIT_DEC_LEN][GAIT_WIDTH];

int GaitMoveBack_Acc[GAIT_ACC_LEN][GAIT_WIDTH];
int GaitMoveBack_Cons[GAIT_CON_LEN][GAIT_WIDTH];
int GaitMoveBack_Dec[GAIT_DEC_LEN][GAIT_WIDTH];

int GaitAutoCalib[GAIT_AUTOCALIB_LEN][GAIT_WIDTH];

int GaitTurnLeft[GAIT_TURN_LEN][GAIT_WIDTH];
int GaitTurnRight[GAIT_TURN_LEN][GAIT_WIDTH];

////////////////////////////////////////////////

const int MapAbsToPhy[18]=
{
    10,	11,	9,
    12,	14,	13,
    17,	15,	16,
    6,	8,	7,
    3,	5,	4,
    0,	2,	1
};
const int MapPhyToAbs[18]=
{
    15,	17,	16,
    12,	14,	13,
    9,	11,	10,
    2,	0,	1,
    3,	5,	4,
    7,	8,	6
};

CGait::CGait()
{
    CGait::IsConsFinished=false;
    CGait::IsAdaptiveGaitRegistered=false;
    CGait::IsAutoCalibrateRegistered = false;
    CGait::IsAutoCalibrateStepRegistered = false;
    CGait::Gait_iter=1;
    CGait::Gait_iter_count=0;
    CGait::m_currentGait=EGAIT::GAIT_NULL;
    CGait::m_gaitState=EGaitState::NONE;
}

CGait::~CGait()
{
}


void CGait::MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& p_data)
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        m_feedbackDataMapped[i].Position=p_data.feedbackData[MapAbsToPhy[i]].Position;
        m_feedbackDataMapped[i].Velocity=p_data.feedbackData[MapAbsToPhy[i]].Velocity;
        m_feedbackDataMapped[i].Torque=p_data.feedbackData[MapAbsToPhy[i]].Torque;
    }
}

void CGait::MapCommandDataOut(Aris::RT_CONTROL::CMachineData& p_data)
{
    for(int i=0;i<MOTOR_NUM;i++)
    {
        p_data.commandData[i].Position=m_commandDataMapped[MapPhyToAbs[i]].Position;
        p_data.commandData[i].Velocity=m_commandDataMapped[MapPhyToAbs[i]].Velocity;
        p_data.commandData[i].Torque=m_commandDataMapped[MapPhyToAbs[i]].Torque;
    }
}

bool CGait::IsGaitFinished()
{
    if(m_gaitState==EGaitState::GAIT_STOP)
        return true;
    else
        return false;
}

static std::ifstream fin;

int CGait::InitGait(Aris::RT_CONTROL::CSysInitParameters& param)
{
    int Line_Num;
    double temp;

    std::cout << "Start Reading File" << std::endl;
    std::cout<<"reading file data..."<<std::endl;

    fin.open("./gait/TL.txt");

    for(int i=0;i<GAIT_TURN_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitTurnLeft[i][j]=-(int)temp;
        }
    }

    fin.close();

    fin.open("./gait/TR.txt");

    for(int i=0;i<GAIT_TURN_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitTurnRight[i][j]=-(int)temp;
        }
    }

    fin.close();

    fin.open("./gait/start.txt");

    for(int i=0;i<GAIT_HOME2START_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitHome2Start[i][j]=-(int)temp;
        }
    }

    fin.close();

    //FOWARD

    fin.open("./gait/acc.txt");

    for(int i=0; i<GAIT_ACC_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitMove_Acc[i][j]=-(int)temp;
        }

    }
    fin.close();

    fin.open("./gait/cons.txt");

    for(int i=0+GAIT_ACC_LEN; i<GAIT_ACC_LEN+GAIT_CON_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitMove_Cons[i-GAIT_ACC_LEN][j]=-(int)temp;
        }

    }
    fin.close();

    fin.open("./gait/dec.txt");

    for(int i=0+GAIT_ACC_LEN+GAIT_CON_LEN; i<GAIT_ACC_LEN+GAIT_CON_LEN+GAIT_DEC_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitMove_Dec[i-GAIT_ACC_LEN-GAIT_CON_LEN][j]=-(int)temp;
        }

    }
    fin.close();

    //BACKWARD

    fin.open("./gait/back_acc.txt");

    for(int i=0; i<GAIT_ACC_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitMoveBack_Acc[i][j]=-(int)temp;
        }
    }

    fin.close();

    fin.open("./gait/back_cons.txt");

    for(int i=0+GAIT_ACC_LEN; i<GAIT_ACC_LEN+GAIT_CON_LEN;i++)
    {
        fin>>Line_Num;

        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitMoveBack_Cons[i-GAIT_ACC_LEN][j]=-(int)temp;
        }
    }

    fin.close();

    fin.open("./gait/back_dec.txt");

    for(int i=0+GAIT_ACC_LEN+GAIT_CON_LEN; i<GAIT_ACC_LEN+GAIT_CON_LEN+GAIT_DEC_LEN;i++)
    {
        fin>>Line_Num;
        for(int j=0;j<GAIT_WIDTH;j++)
        {
            fin>>temp;
            GaitMoveBack_Dec[i-GAIT_ACC_LEN-GAIT_CON_LEN][j]=-(int)temp;
        }
    }

    fin.close();

    return 0;
}

int CGait::RunGait(EGAIT& p_gait,Aris::RT_CONTROL::CMachineData& p_data)
{

    MapFeedbackDataIn(p_data);

    switch(p_gait)
    {

    case GAIT_NULL:
        rt_printf("GAIT_NULL will transfer to GAIT_STANDSTILL\n");
        p_gait=GAIT_STANDSTILL;
        CGait::m_gaitState=EGaitState::GAIT_STOP;
        for(int i=0;i<MOTOR_NUM;i++)
        {
            m_standStillData[i].Position=m_feedbackDataMapped[i].Position;
            m_standStillData[i].Velocity=m_feedbackDataMapped[i].Velocity;
            m_standStillData[i].Torque=m_feedbackDataMapped[i].Torque;

            m_commandDataMapped[i].Position=m_standStillData[i].Position;
            m_commandDataMapped[i].Velocity=m_standStillData[i].Velocity;
            m_commandDataMapped[i].Torque=m_standStillData[i].Torque;
        }
        rt_printf("GAIT_NONE will transfer to GAIT_STANDSTILL...");
        break;

    case GAIT_STANDSTILL:
        if(p_gait!=m_currentGait)
        {
            rt_printf("GAIT_STANDSTILL begin\n");
            m_currentGait=p_gait;
            m_gaitStartTime=p_data.time;
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_standStillData[i].Position=m_feedbackDataMapped[i].Position;
                m_standStillData[i].Velocity=m_feedbackDataMapped[i].Velocity;
                m_standStillData[i].Torque=m_feedbackDataMapped[i].Torque;

                m_commandDataMapped[i].Position=m_standStillData[i].Position;
                m_commandDataMapped[i].Velocity=m_standStillData[i].Velocity;
                m_commandDataMapped[i].Torque=m_standStillData[i].Torque;
            }
        }
        else
        {
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=m_standStillData[i].Position;
                m_commandDataMapped[i].Velocity=m_standStillData[i].Velocity;
                m_commandDataMapped[i].Torque=m_standStillData[i].Torque;
            }
        }
        break;

    case GAIT_MOVE:

        if(p_gait!=m_currentGait)
        {
            rt_printf("GAIT_MOVE begin\n");
            m_gaitState=EGaitState::GAIT_RUN;
            m_currentGait=p_gait;
            m_gaitStartTime=p_data.time;
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=GaitMove_Acc[0][i];
            }
            rt_printf("Begin Accelerating foward...");
        }
        else
        {
            m_gaitCurrentIndex=(int)(p_data.time-m_gaitStartTime);
            rt_printf("index%d and Gait_iter_count%d and Gait_iter%d\n",m_gaitCurrentIndex,Gait_iter_count,Gait_iter);

            if(m_gaitCurrentIndex<GAIT_ACC_LEN)
            {
                for(int i=0;i<MOTOR_NUM;i++)
                {
                    m_commandDataMapped[i].Position=GaitMove_Acc[m_gaitCurrentIndex][i];
                }
            }

            else
            {
                if(Gait_iter>=1&&IsConsFinished==false)
                {
                    for(int i=0;i<MOTOR_NUM;i++)
                    {
                        m_commandDataMapped[i].Position=GaitMove_Cons[(m_gaitCurrentIndex-GAIT_ACC_LEN)%GAIT_CON_LEN][i];
                    }
                    if(((m_gaitCurrentIndex-GAIT_ACC_LEN)%GAIT_CON_LEN)==0)
                    {
                        Gait_iter_count++;
                    }
                    if(((m_gaitCurrentIndex-GAIT_ACC_LEN)%GAIT_CON_LEN)==3999)
                    {
                        Gait_iter--;
                    }
                }
                else
                {
                    IsConsFinished=true;
                    for(int i=0;i<MOTOR_NUM;i++)
                    {
                        m_commandDataMapped[i].Position=GaitMove_Dec[(m_gaitCurrentIndex-GAIT_ACC_LEN-GAIT_CON_LEN*Gait_iter_count)][i];
                    }
                }

                if(m_gaitCurrentIndex==GAIT_ACC_LEN+GAIT_CON_LEN*Gait_iter_count+GAIT_DEC_LEN-1)
                {
                    rt_printf("GAIT_MOVE will transfer to GAIT_STANDSTILL...");

                    p_gait=GAIT_STANDSTILL;

                    for(int i=0;i<MOTOR_NUM;i++)
                    {
                        m_standStillData[i].Position=m_feedbackDataMapped[i].Position;
                        m_standStillData[i].Velocity=m_feedbackDataMapped[i].Velocity;
                        m_standStillData[i].Torque=m_feedbackDataMapped[i].Torque;
                    }
                    //only in this cycle, out side get true from IsGaitFinished()
                    m_gaitState=EGaitState::GAIT_STOP;
                    Gait_iter_count=0;
                    Gait_iter=1;
                    IsConsFinished=false;
                }
            }
        }
        break;

    case GAIT_MOVE_BACK:

        if(p_gait!=m_currentGait)
        {
            rt_printf("GAIT_MOVE_back begin\n");
            m_gaitState=EGaitState::GAIT_RUN;
            m_currentGait=p_gait;
            m_gaitStartTime=p_data.time;
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=GaitMoveBack_Acc[0][i];
            }
            rt_printf("Begin Accelerating foward...");
        }
        else
        {
            m_gaitCurrentIndex=(int)(p_data.time-m_gaitStartTime);
            rt_printf("index%d and Gait_iter_count%d and Gait_iter%d\n",m_gaitCurrentIndex,Gait_iter_count,Gait_iter);

            if(m_gaitCurrentIndex<GAIT_ACC_LEN)
            {
                for(int i=0;i<MOTOR_NUM;i++)
                {
                    m_commandDataMapped[i].Position=GaitMoveBack_Acc[m_gaitCurrentIndex][i];
                }
            }

            else
            {
                if(Gait_iter>=1&&IsConsFinished==false)
                {
                    for(int i=0;i<MOTOR_NUM;i++)
                    {
                        m_commandDataMapped[i].Position=GaitMoveBack_Cons[(m_gaitCurrentIndex-GAIT_ACC_LEN)%GAIT_CON_LEN][i];
                    }
                    if(((m_gaitCurrentIndex-GAIT_ACC_LEN)%GAIT_CON_LEN)==0)
                    {
                        Gait_iter_count++;
                    }
                    if(((m_gaitCurrentIndex-GAIT_ACC_LEN)%GAIT_CON_LEN)==3999)
                    {
                        Gait_iter--;
                    }
                }
                else
                {
                    IsConsFinished=true;
                    for(int i=0;i<MOTOR_NUM;i++)
                    {
                        m_commandDataMapped[i].Position=GaitMoveBack_Dec[(m_gaitCurrentIndex-GAIT_ACC_LEN-GAIT_CON_LEN*Gait_iter_count)][i];
                    }
                }

                if(m_gaitCurrentIndex==GAIT_ACC_LEN+GAIT_CON_LEN*Gait_iter_count+GAIT_DEC_LEN-1)
                {
                    rt_printf("GAIT_MOVE will transfer to GAIT_STANDSTILL...");

                    p_gait=GAIT_STANDSTILL;

                    for(int i=0;i<MOTOR_NUM;i++)
                    {
                        m_standStillData[i].Position=m_feedbackDataMapped[i].Position;
                        m_standStillData[i].Velocity=m_feedbackDataMapped[i].Velocity;
                        m_standStillData[i].Torque=m_feedbackDataMapped[i].Torque;
                    }

                    m_gaitState=EGaitState::GAIT_STOP;
                    Gait_iter_count=0;
                    Gait_iter=1;
                    IsConsFinished=false;
                }
            }
        }
        break;

    case GAIT_HOME2START:

        if(p_gait!=m_currentGait)
        {
            rt_printf("GAIT_HOME2START begin\n");
            m_gaitState=EGaitState::GAIT_RUN;
            m_currentGait=p_gait;
            m_gaitStartTime=p_data.time;
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=GaitHome2Start[0][i];
            }

        }
        else
        {
            m_gaitCurrentIndex=(int)(p_data.time-m_gaitStartTime);
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=GaitHome2Start[m_gaitCurrentIndex][i];
            }

            if(m_gaitCurrentIndex==GAIT_HOME2START_LEN-1)
            {
                rt_printf("GAIT_HOME2START will transfer to GAIT_STANDSTILL...");
                p_gait=GAIT_STANDSTILL;
                for(int i=0;i<MOTOR_NUM;i++)
                {
                    m_standStillData[i].Position=m_feedbackDataMapped[i].Position;
                    m_standStillData[i].Velocity=m_feedbackDataMapped[i].Velocity;
                    m_standStillData[i].Torque=m_feedbackDataMapped[i].Torque;
                }
                m_gaitState=EGaitState::GAIT_STOP;
            }
        }
        break;

    case GAIT_AUTO_CALIBRATION:
        if(p_gait!=m_currentGait)
        {
            rt_printf("Auto Calibrating!!!\n");
            m_gaitState=EGaitState::GAIT_RUN;
            m_currentGait=p_gait;
            m_gaitStartTime=p_data.time;
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=CGait::GaitAutoCalib[0][i];
                //rt_printf("First Line %d\n",GaitAutoCalib[0][MOTOR_NUM-1]);
            }
        }
        else
        {
            m_gaitCurrentIndex=(int)(p_data.time-m_gaitStartTime);

            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=CGait::GaitAutoCalib[m_gaitCurrentIndex][i];

            }

            if(m_gaitCurrentIndex==GAIT_AUTOCALIB_LEN-1)
            {
                //rt_printf("Other Line %d\n",GaitAutoCalib[m_gaitCurrentIndex][MOTOR_NUM-1]);
                rt_printf("GAIT_AUTOCALIBRATION will transfer to GAIT_STANDSTILL...\n");
                p_gait=GAIT_STANDSTILL;
                for(int i=0;i<MOTOR_NUM;i++)
                {
                    m_standStillData[i].Position=m_feedbackDataMapped[i].Position;
                    m_standStillData[i].Velocity=m_feedbackDataMapped[i].Velocity;
                    m_standStillData[i].Torque=m_feedbackDataMapped[i].Torque;
                }
                m_gaitState=EGaitState::GAIT_STOP;
                IsAutoCalibrateStepRegistered = true;
            }
        }
        break;


    case GAIT_TURN_LEFT:

        if(p_gait!=m_currentGait)
        {
            rt_printf("GAIT_TURNLEFT begin\n");
            m_gaitState=EGaitState::GAIT_RUN;
            m_currentGait=p_gait;
            m_gaitStartTime=p_data.time;
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=GaitTurnLeft[0][i];
            }

        }
        else
        {
            m_gaitCurrentIndex=(int)(p_data.time-m_gaitStartTime);
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=GaitTurnLeft[m_gaitCurrentIndex][i];
            }

            if(m_gaitCurrentIndex==GAIT_TURN_LEN-1)
            {
                rt_printf("GAIT_TURN will transfer to GAIT_STANDSTILL...");
                p_gait=GAIT_STANDSTILL;
                for(int i=0;i<MOTOR_NUM;i++)
                {
                    m_standStillData[i].Position=m_feedbackDataMapped[i].Position;
                    m_standStillData[i].Velocity=m_feedbackDataMapped[i].Velocity;
                    m_standStillData[i].Torque=m_feedbackDataMapped[i].Torque;
                }
                m_gaitState=EGaitState::GAIT_STOP;
            }
        }
        break;

    case GAIT_TURN_RIGHT:

        if(p_gait!=m_currentGait)
        {
            rt_printf("GAIT_TURNRIGHT begin\n");
            m_gaitState=EGaitState::GAIT_RUN;
            m_currentGait=p_gait;
            m_gaitStartTime=p_data.time;
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=GaitTurnRight[0][i];
            }
        }
        else
        {
            m_gaitCurrentIndex=(int)(p_data.time-m_gaitStartTime);
            for(int i=0;i<MOTOR_NUM;i++)
            {
                m_commandDataMapped[i].Position=GaitTurnRight[m_gaitCurrentIndex][i];
            }

            if(m_gaitCurrentIndex==GAIT_TURN_LEN-1)
            {
                rt_printf("GAIT_turn will transfer to GAIT_STANDSTILL...");
                p_gait=GAIT_STANDSTILL;
                for(int i=0;i<MOTOR_NUM;i++)
                {
                    m_standStillData[i].Position=m_feedbackDataMapped[i].Position;
                    m_standStillData[i].Velocity=m_feedbackDataMapped[i].Velocity;
                    m_standStillData[i].Torque=m_feedbackDataMapped[i].Torque;
                }
                m_gaitState=EGaitState::GAIT_STOP;
            }
        }
        break;
    default:
        break;
    }
    MapCommandDataOut(p_data);

    return 0;
}
