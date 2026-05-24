#include "globals.h"

// ============================================================
//  Shared global variable definitions
//  All translation units include globals.h for the extern decls;
//  memory is allocated exactly once here.
// ============================================================

// ----- Timing -----
float previousTime = 0.0f, deltaTime = 0.0f, currentTime = 0.0f;

// ----- IMU -----
int16_t rawGyroX  = 0, rawGyroY  = 0, rawGyroZ  = 0;
int16_t rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;
float   accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
float   gyroX  = 0.0f, gyroY  = 0.0f, gyroZ  = 0.0f;
float   accelPitch = 0.0f, accelRoll = 0.0f;
float   pitch  = 0.0f, roll = 0.0f, yaw = 0.0f;
float   alphaIMU = 0.98f;  // complementary filter coefficient for IMU angle estimation

// ============================================================
//  Mahony filter gains — tune on the bench
//  KP: how aggressively the filter corrects toward the accelerometer.
//      Too high = accelerometer noise bleeds in during throttle changes.
//      Too low  = gyro drift builds up between corrections.
//  KI: how fast the gyro bias is estimated and cancelled.
//      Keep small. Set to 0 first, add back only if you see slow drift
//      over 30+ seconds of hover.
// ============================================================

float MAHONY_KP = 0.5f;
float MAHONY_KI = 0.5f;

// ----- Barometer -----
float    PAR_T1 = 0, PAR_T2 = 0, PAR_T3 = 0;
float    PAR_P1 = 0, PAR_P2 = 0, PAR_P3  = 0, PAR_P4  = 0;
float    PAR_P5 = 0, PAR_P6 = 0, PAR_P7  = 0, PAR_P8  = 0;
float    PAR_P9 = 0, PAR_P10 = 0, PAR_P11 = 0;
uint32_t rawPressure = 0, rawTemp = 0;
float    pressure = 0.0f, temp = 0.0f, altitude = 0.0f, hoverAltitude = 0.0f;
float    initialTemp = 0.0f, initialPressure = 0.0f;
bool     altitudeLock = false;



// ----- Receiver / CRSF -----
uint8_t channelsPacket[26] = {0};
uint8_t data0[22]          = {0};
uint8_t crc8_last          = 0;   // BUG FIX: was 'crc8' — see globals.h note

// ----- Battery -----
int16_t  rawBatteryReadings       = 0;
float    measuredVoltage          = 0.0f;
float    batteryVoltage           = 0.0f;
float    batteryPercentUsed       = 0.0f;
uint32_t batteryCapacityRemaining = 0;  //mAh
uint8_t  batteryRemainingPercent  = 0;
const float V_REF = (MAX_BATTERY_VOLTAGE - (R1 * MAX_BATTERY_VOLTAGE) / (R2 + R1));  //voltage reference at max battery voltage after voltage divider

// ----- Control references -----
int desiredPitch = 0, desiredRoll = 0, yawSpeed = 0, desiredAltitude = 0;
int mainThrottleInput = 0;

// ----- PID outputs -----
float rollAnglePID  = 0.0f, pitchAnglePID = 0.0f;
float rollRatePID   = 0.0f, pitchRatePID  = 0.0f, yawRatePID = 0.0f;
float altitudePID   = 0.0f;

// ----- Motor outputs -----
int motorMatrix[4] = {0, 0, 0, 0};

// ----- PID controllers -----
// BUG FIX: PID compute() signature now takes only (ref, fb, dt) — the
//          'currentActuatorInput' parameter was accepted but never used inside
//          compute(); removing it eliminates a confusing dead argument.
PIDController rollAngleController (1.0f,   0.0f,   0.0f,    0.0f);
PIDController pitchAngleController(1.0f,   0.0f,   0.0f,    0.0f);
PIDController rollRateController  (0.224f, 0.7923f, 0.0019f, 0.0f);
PIDController pitchRateController (0.224f, 0.7923f, 0.0019f, 0.0f);
PIDController yawRateController   (0.2689f,0.5704f, 0.0038f, 0.0f);
PIDController altitudeController  (0.1f,   0.1f,   0.1f,    0.0f);
