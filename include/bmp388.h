#pragma once
#include "globals.h"
#include <Wire.h>

void initBMP388();
void getInitialPressure(int sampleSize);
void updateAltitudeReadings();
void getCompensationData();
void checkSensorStatus();
void readTempPres(uint32_t *p, uint32_t *t);
float BMP388CompensateTemp(uint32_t uncompTemp);       // BUG FIX: was 'CompensatTemp'
float BMP388CompensatePressure(uint32_t uncompPressure, float calibratedTemp);
