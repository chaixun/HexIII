#ifndef GAIT_H
#define GAIT_H

#include <fstream>
#include <iostream>
#include <Aris_Control.h>
#include <Aris_ControlData.h>

using namespace std;

#define GAIT_WIDTH 18

#define GAIT_MOVE_LEN 3700

#define GAIT_HOME2START_LEN 4001

#define GAIT_TURN_LEN 6001

#define GAIT_ADAPTIVEWALK_LEN 12000

#define GAIT_DISCOVER_LEN 2500

#define MOTOR_NUM 18

enum EGaitState
{
    NONE,
    GAIT_START,
    GAIT_RUN,
    GAIT_STOP,
};

enum EGAIT
{
    GAIT_NULL=0,
    GAIT_STANDSTILL=1,
    GAIT_HOME2START=2,
    GAIT_MOVE=3,
    GAIT_MOVE_BACK=4,
    GAIT_TURN_LEFT=5,
    GAIT_TURN_RIGHT=6,
    GAIT_WALK_ADAPTIVE = 7,
    GAIT_BEGIN_DISCOVER = 8,
    GAIT_END_DISCOVER = 9,
};

class CGait
{
public:
    CGait();
    ~CGait();
    static int InitGait(Aris::RT_CONTROL::CSysInitParameters& param);
    static int RunGait(EGAIT& p_gait, Aris::RT_CONTROL::CMachineData& p_data);
    static bool IsGaitFinished();
    static bool IsWalkAdaptiveRegistered;
    static bool IsWalkAdaptiveStepRegistered;
    static bool IsWalkAvoidRegistered;
    static bool IsWalkAvoidStepRegistered;
    static int GaitAdaptiveWalk[GAIT_ADAPTIVEWALK_LEN][GAIT_WIDTH];

private:
    static EGAIT m_currentGait;
    static long long int m_gaitStartTime;
    static int m_gaitCurrentIndex;
    static EGaitState m_gaitState;
    static Aris::RT_CONTROL::CMotorData m_standStillData[MOTOR_NUM];
    static Aris::RT_CONTROL::CMotorData m_commandDataMapped[MOTOR_NUM];
    static Aris::RT_CONTROL::CMotorData m_feedbackDataMapped[MOTOR_NUM];
    static void MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& p_data);
    static void MapCommandDataOut(Aris::RT_CONTROL::CMachineData& p_data);

};

#endif // GAIT_H
