#pragma once
#include <Arduino.h>

// ===== Wi-Fi credentials =====
// Defined in wifi_server.cpp — only declared here
extern const char *ssid;
extern const char *password;

// ===== BMP388 =====
#define BMP388ADDRESS  0x76
#define I2C_SDA_PIN    21
#define I2C_SCL_PIN    22
#define i2cSpeed       1000000
#define P0             101325
#define InitSampleSize 200

// ===== ICM45686 =====
#define ICMAddress       0x68
#define accelSensitivity 8192
#define gyroSensitivity  65.536

// ===== CRSF / Receiver =====
#define CRSF_Addr             0xC8
#define CRSF_Sensor           0xEA
#define CRSF_Type_Battery     0x08
#define CRSF_Type_Altitude    0x09
#define CRSF_Type_Channels    0x16
#define CRSF_Type_Attitude    0x1E
#define CRSF_Length_Channels  0x18
#define CRSF_Length_Altitude  0x6
#define Rx 16   //UART Rx pin for the receiver
#define Tx 17   //UART Tx pin for the receiver
#define bRate 416666    //CRSF baud rate

// ===== Motor / PWM pins =====
#define frontLeftPin  32    //M1 Green Wire
#define frontRightPin 25    //M2 Red Wire
#define rearRightPin  26    //M3 Orange Wire
#define rearLeftPin   33    //M4 Yellow Wire
#define frontLeft     1
#define frontRight    2
#define rearRight     3
#define rearLeft      4

// ===== PWM / signal =====
#define PWMFrequency  500
#define PWMResolution 11
#define PWMRange      pow(2.0, PWMResolution)
#define signalMax     1811              //max raw signal value
#define signalMin     174               //min raw signal value
#define minThrottleRequirement 0.525    //5% Throttle
#define maxMainThrottleAllowed 0.9      //80% Throttle

// ===== Flight limits =====
#define tiltAngleMax      25
#define yawSpeedMax       90
#define minAltitude       0
#define maxAltitude       30
#define maxThrottleValue  PWMRange - 1  //100% Throttle
#define minThrottleValue  (PWMRange/2) - 1  //0% Throttle

// ===== Battery =====
#define batteryADCPin      36
#define R1                 10000
#define R2                 1980
#define resistanceRatio    ((R1 + R2) / R2)
#define Vref               2.8
#define maxBatteryVoltage  16.8
#define maxBatteryCapacity 10400

// ============================================================
//  PID Controller struct
// ============================================================

struct PIDController
{
    float Kp = 0, Ki = 0, Kd = 0;

    float derivativeFilterAlpha = 0.95;
    float tolerance;
    float integralMax, integralMin;

    float error                  = 0;
    float reference              = 0;
    float feedbackState          = 0;
    float previousFeedbackState  = 0;
    float integralTerm           = 0;
    float derivativeTermRaw      = 0;
    float derivativeTermFiltered = 0;
    // float integralStartPoint     = 0.55;
    float PID_output             = 0;
    float maxActuatorOutput      = maxThrottleValue;

    PIDController(float kp, float ki, float kd, float tol)
    {
        Kp                = kp;
        Ki                = ki;
        Kd                = kd;
        // integralStartPoint = iStPt;
        tolerance         = tol;
    }

    float compute(float reference, float feedback, float dt, float currentActuatorInput)
    {
        error = reference - feedback;
        if (abs(error) > tolerance)
        {
            derivativeTermRaw      = -(feedback - previousFeedbackState) / dt;
            derivativeTermFiltered = derivativeFilterAlpha * derivativeTermFiltered
                                   + (1.0 - derivativeFilterAlpha) * derivativeTermRaw;

            integralTerm = integralTerm + error * dt;

            PID_output = (Kp * error) + (Ki * integralTerm) + (Kd * derivativeTermFiltered);
        }
        previousFeedbackState = feedback;
        return PID_output;
    }

    void reset()
    {
        error                  = 0;
        integralTerm           = 0;
        derivativeTermRaw      = 0;
        derivativeTermFiltered = 0;
        PID_output             = 0;
    }

    float unwind(float dt)
    {
        integralTerm = integralTerm - error * dt;
        return PID_output = (Kp * error) + (Ki * integralTerm) + (Kd * derivativeTermFiltered);
    }
};

// ============================================================
//  Shared globals — defined in main.cpp
// ============================================================

// Timing
extern float previousTime, deltaTime, currentTime;

// IMU
extern int16_t rawGyroX, rawGyroY, rawGyroZ;
extern int16_t rawAccelX, rawAccelY, rawAccelZ;
extern float accelX, accelY, accelZ;
extern float gyroX, gyroY, gyroZ;
extern float accelPitch, accelRoll;
extern float pitch, roll, yaw;
extern float alphaIMU;

// Barometer
extern uint8_t  altCounter;
extern float    PAR_T1, PAR_T2, PAR_T3;
extern float    PAR_P1, PAR_P2, PAR_P3, PAR_P4, PAR_P5, PAR_P6;
extern float    PAR_P7, PAR_P8, PAR_P9, PAR_P10, PAR_P11;
extern uint32_t rawPressure, rawTemp;
extern float    pressure, temp, altitude, hoverAltitude;
extern float    initialTemp, initialPressure, altitude1;
extern bool     altitudeLock;

// Receiver / channels raw packet
extern uint8_t channelsPacket[26];
extern uint8_t data0[22];
extern uint8_t crc8;

// Battery
extern int16_t rawBatteryReadings;
extern float   measuredVoltage, batteryVoltage, batteryPercentUsed;
extern uint32_t batteryCapacityRemaining;
extern uint8_t  batteryRemainingPercent;

// Control loop references
extern int   desiredPitch, desiredRoll, yawSpeed, desiredAltitude;
extern int   mainThrottleInput;
extern float rollAnglePID, pitchAnglePID;
extern float rollRatePID, pitchRatePID, yawRatePID;
extern float altitudePID;
extern int   motorMatrix[4];

// PID controllers — defined in main.cpp
extern PIDController rollAngleController;
extern PIDController pitchAngleController;
extern PIDController rollRateController;
extern PIDController pitchRateController;
extern PIDController yawRateController;
extern PIDController altitudeController;
