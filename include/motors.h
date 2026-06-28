#pragma once
#include "globals.h"

extern bool manualControlled;

bool useIntegral();
void movement();
void updateRatePIDLoop(bool integralActive);
void updateOuterPIDLoop(bool integralActive);
bool determineFlightControlMode();
void motorMixingAlgorithm(bool altitudeControlled);
void anti_Windup();
void clampMixedMotors();
void resetControllers();
