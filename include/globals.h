#pragma once
#include "config.h"
#include "pid.h"

// ============================================================
//  All shared globals — declared extern here, DEFINED in globals.cpp
//  Include this header wherever a translation unit needs them.
//  Do NOT include config.h for globals — use this file instead.
// ============================================================

// ----- Timing -----
extern float previousTime, deltaTime, currentTime;

// ----- IMU -----
extern int16_t rawGyroX,  rawGyroY,  rawGyroZ;
extern int16_t rawAccelX, rawAccelY, rawAccelZ;
extern float   accelX, accelY, accelZ;
extern float   gyroX,  gyroY,  gyroZ;
extern float   accelPitch, accelRoll;
extern float   pitch, roll, yaw;
extern float   alphaIMU;
extern float MAHONY_KP, MAHONY_KI;  // Mahony filter gains

// ----- Barometer -----
extern float    PAR_T1, PAR_T2, PAR_T3;
extern float    PAR_P1, PAR_P2, PAR_P3,  PAR_P4,  PAR_P5,  PAR_P6;
extern float    PAR_P7, PAR_P8, PAR_P9,  PAR_P10, PAR_P11;
extern uint32_t rawPressure, rawTemp;
extern float    pressure, temp, altitude, hoverAltitude;
extern float    initialTemp, initialPressure;
extern bool     altitudeLock;
extern float    altitudeFromAccel, velocityZ, altitudeComplementaryAlpha;

// ----- Receiver / CRSF -----
extern uint8_t channelsPacket[26];
extern uint8_t data0[22];
extern uint8_t crc8_last;   // BUG FIX: renamed from 'crc8' which collides with the
                             // crc8_d5() function name in some compilers/linkers and
                             // is a reserved identifier prefix in some C standards.

// ----- Battery -----
extern int16_t  rawBatteryReadings;
extern float    measuredVoltage, batteryVoltage, batteryPercentUsed;
extern uint32_t batteryCapacityRemaining;
extern uint8_t  batteryRemainingPercent;

// ----- Control references -----
extern int   desiredPitch, desiredRoll, yawSpeed, desiredAltitude;
extern int   mainThrottleInput;

// ----- PID outputs -----
extern float rollAnglePID,  pitchAnglePID;
extern float rollRatePID,   pitchRatePID,  yawRatePID;
extern float altitudePID;

// ----- Motor outputs -----
extern int motorMatrix[4];

// ----- PID controllers -----
extern PIDController rollAngleController;
extern PIDController pitchAngleController;
extern PIDController rollRateController;
extern PIDController pitchRateController;
extern PIDController yawRateController;
extern PIDController altitudeController;
