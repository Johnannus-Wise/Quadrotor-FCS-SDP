#pragma once
#include "globals.h"

void movement();
bool updatePID();
void motorMixingAlgorithm(bool altitudeControlled);
void anti_Windup();
void clampMixedMotors();
