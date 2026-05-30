#pragma once

#include "Config.h"

void send_telemetry(const SystemData &data, RobotState state);
void TaskSensorCode(void * pvParameters);

#