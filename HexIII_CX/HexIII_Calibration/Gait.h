#ifndef GAIT_H
#define GAIT_H

#include <fstream>
#include <iostream>
#include <Aris_Control.h>
#include <Aris_ControlData.h>

using namespace std;

#define GAIT_WIDTH 18

#define GAIT_ACC_LEN 2001
#define GAIT_CON_LEN 4000
#define GAIT_DEC_LEN 2000

#define GAIT_HOME2START_LEN 4001

#define GAIT_AUTOCALIB_LEN 4000

#define GAIT_TURN_LEN 6001

//for test
//#define GAIT_HOME2START_LEN 3700

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
    GAIT_AUTO_CALIBRATION = 5,
    GAIT_TURN_LEFT=6,
    GAIT_TURN_RIGHT=7,
};

class CGait
{
public:
    CGait();
    ~CGait();
    static int InitGait(Aris::RT_CONTROL::CSysInitParameters& param);
    static int RunGait(EGAIT& p_gait, Aris::RT_CONTROL::CMachineData& p_data);
    static bool IsGaitFinished();
    static bool IsHomeStarted;
    static bool IsConsFinished;
    static bool IsAdaptiveGaitRegistered;
    static bool IsAutoCalibrateRegistered;
    static bool IsAutoCalibrateStepRegistered;
    static int Gait_iter;
    static int Gait_iter_count;

    static int GaitAutoCalib[GAIT_AUTOCALIB_LEN][GAIT_WIDTH];

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
