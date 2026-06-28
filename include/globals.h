#pragma once
#include "config.h"
#include "pid.h"

// ============================================================
//  All shared globals — declared extern here, DEFINED in globals.cpp
//  Include this header wherever a translation unit needs them.
//  Do NOT include config.h for globals — use this file instead.
// ============================================================

// ----- Timing -----
// Legacy single deltaTime — still used by Mahony filter in icm45686.cpp
// (the IMU update is called from the rate loop, so dtRate == deltaTime there).
// New per-loop deltas are defined in main.cpp and declared here for motors.cpp.

// ===== Update Rates =====
extern uint32_t ATTITUDE_UPDATE_RATE, ALTITUDE_UPDATE_RATE, PACKET_UPDATE_RATE, WEB_HANDLE_UPDATE_RATE, IMU_UPDATE_RATE;

extern float previousTime, deltaTime, currentTime;
extern float dtAttitude;       // inner rate loop   (500 Hz)
extern float dtAltitude;   // altitude loop     ( 50 Hz)

extern uint32_t now;       // Current timestamp (micros()) updated each loop
extern uint32_t attitudeLoopLast, altitudeLoopLast, flightUpdateLast;
extern uint32_t packetUpdateLast, webHandleLast, imuUpdateLast;
extern const float ATTITUDE_LOOP_US, ALTITUDE_LOOP_US, FLIGHT_UPDATE_US;
extern const float PACKET_UPDATE_US, WEB_HANDLE_US, IMU_UPDATE_US;

// ----- IMU -----
extern int16_t rawGyroX,  rawGyroY,  rawGyroZ;
extern int16_t rawAccelX, rawAccelY, rawAccelZ;
extern float   accelX, accelY, accelZ;
extern float accelX_filtered, accelY_filtered, accelZ_filtered;
extern float   gyroX,  gyroY,  gyroZ;
extern float   accelPitch, accelRoll;
extern float   pitch, roll, yaw;
extern float   ACCEL_LPF_ALPHA;  // Low-pass filter coefficient for accelerometer scaled accelerometer readings.
extern float MAHONY_KP, MAHONY_KI;  // Mahony filter gains

// ----- Barometer -----
extern float    PAR_T1, PAR_T2, PAR_T3;
extern float    PAR_P1, PAR_P2, PAR_P3,  PAR_P4,  PAR_P5,  PAR_P6;
extern float    PAR_P7, PAR_P8, PAR_P9,  PAR_P10, PAR_P11;
extern uint32_t rawPressure, rawTemp;
extern float    pressure, temp, altitude, hoverAltitude;
extern float    initialTemp, initialPressure;
extern bool     altitudeLock;

// ----- Receiver / CRSF -----
extern uint8_t telemetrySelector;
extern uint8_t channelsPacket[26];
extern uint8_t crc8_last;   // BUG FIX: renamed from 'crc8' which collides with the
                             // crc8_d5() function name in some compilers/linkers and
                             // is a reserved identifier prefix in some C standards.

// ----- Battery -----
extern int16_t  rawBatteryReadings;
extern float    measuredVoltage, batteryVoltage, batteryPercentUsed;
extern uint32_t batteryCapacityRemaining;
extern uint8_t  batteryRemainingPercent;
extern const float V_REF;

// ----- Control references -----
extern int   desiredPitch, desiredRoll, yawSpeed, desiredAltitude;
extern int   mainThrottleInput;

// ----- PID outputs -----
extern float rollAnglePID,  pitchAnglePID;
extern float rollRatePID,   pitchRatePID,  yawRatePID;
extern float altitudePID;

// ----- Motor outputs -----
extern int MAX_THROTTLE_VALUE;
extern int MIN_THROTTLE_VALUE;
extern int motorMatrix[4];
extern float throttlePercentage;

// ----- PID controllers -----
extern PIDController rollAngleController;
extern PIDController pitchAngleController;
extern PIDController rollRateController;
extern PIDController pitchRateController;
extern PIDController yawRateController;
extern PIDController altitudeController;
