#ifndef SENSORTASK_H
#define SENSORTASK_H

#include "Config.h"

void send_telemetry(const SystemData &data, RobotState state);
void TaskSensorCode(void * pvParameters);

#endif