#ifndef FSMTASK_H
#define FSMTASK_H

#include "Config.h"

void TaskFSMCode(void * pvParameters);
void enterState(RobotState newState);

#endif