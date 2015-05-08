#ifndef CONTROL_H
#define CONTROL_H
#include <iostream>
#include <Aris_Message.h>
#include <Aris_Socket.h>
#include <Aris_Core.h>

#include "Gait.h"
#include "Server.h"

using namespace std;

extern Aris::RT_CONTROL::ACTUATION cs;

static CGait gait;
static EGAIT gaitcmd;

void* Thread1(void *);

int initFun(Aris::RT_CONTROL::CSysInitParameters& param);

int tg(Aris::RT_CONTROL::CMachineData& data);

int autoH2R(void * arg);

int autoPowerOff(void*);

int autoEnable(Aris::Core::MSG &msg);

int autoHome(Aris::Core::MSG &msg);

int autoStart(Aris::Core::MSG &msg);

int autoStop(Aris::Core::MSG &msg);

int autoMove(Aris::Core::MSG &msg);

int autoMoveBack(Aris::Core::MSG &msg);

int autoAvoid(Aris::Core::MSG &msg);

int autoTurnLeft(Aris::Core::MSG &msg);

int autoTurnRight(Aris::Core::MSG &msg);


#endif // CONTROL_H
