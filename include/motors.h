#pragma once
#include "config.h"

void movement();
bool updatePID();
void motorMixingAlgorithm(bool altitudeControlled);
void anti_Windup();
void clampMixedMotors();
