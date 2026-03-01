#pragma once
#include "config.h"
#include <Wire.h>

void initBMP388();
void getInitialPressure(int sampleSize);
void updateAltitudeReadings();
void getCompensationData();
void checkSensorStatus();
void readTempPres(uint32_t *p, uint32_t *t);
float BMP388CompensatTemp(uint32_t uncompTemp);
float BMP388CompensatePressure(uint32_t uncompPressure, float calibratedTemp);
