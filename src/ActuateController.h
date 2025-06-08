// ActuateController.h
#ifndef ACTUATE_CONTROLLER_H
#define ACTUATE_CONTROLLER_H
#include <Arduino.h>

void setupActuators();
void setActuatorPWM(uint8_t actuator, int inputAngle);

#endif