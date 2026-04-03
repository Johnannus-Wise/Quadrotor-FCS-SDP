#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "bmp388.h"
#include "icm45686.h"
#include "receiver.h"
#include "motors.h"
#include "wifi_server.h"

// ============================================================
//  Global variable definitions
//  (declared extern in config.h — memory allocated here ONLY)
// ============================================================

// Timing
float previousTime = 0, deltaTime = 0, currentTime = 0;
int printCount = 0;

// IMU
int16_t rawGyroX = 0,  rawGyroY = 0,  rawGyroZ = 0;
int16_t rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX  = 0, gyroY  = 0, gyroZ  = 0;
float accelPitch = 0.0f, accelRoll = 0.0f;
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float alphaIMU = 0.99;

// Barometer
uint8_t  altCounter = 0;
float    PAR_T1, PAR_T2, PAR_T3;
float    PAR_P1, PAR_P2, PAR_P3, PAR_P4, PAR_P5, PAR_P6;
float    PAR_P7, PAR_P8, PAR_P9, PAR_P10, PAR_P11;
uint32_t rawPressure = 0, rawTemp = 0;
float    pressure = 0, temp = 0, altitude = 0, hoverAltitude = 0;
float    initialTemp = 0, initialPressure = 0, altitude1 = 0;
bool     altitudeLock = false;

// Receiver raw packet buffers
uint8_t channelsPacket[26] = {0};
uint8_t data0[22]          = {0};
uint8_t crc8               = 0;

// Battery
int16_t  rawBatteryReadings    = 0;
float    measuredVoltage       = 0.0;
float    batteryVoltage        = 0.0;
float    batteryPercentUsed    = 0.0;
uint32_t batteryCapacityRemaining = 0;
uint8_t  batteryRemainingPercent  = 0;

// Control references
int desiredPitch = 0, desiredRoll = 0, yawSpeed = 0, desiredAltitude = 0;
int mainThrottleInput = 0;

// PID outputs
float rollAnglePID = 0, pitchAnglePID = 0;
float rollRatePID  = 0, pitchRatePID  = 0, yawRatePID = 0;
float altitudePID  = 0;

// Motor outputs
int motorMatrix[4] = {0, 0, 0, 0};

// PID controllers
PIDController rollAngleController (1, 0.0, 0.0, 0);
PIDController pitchAngleController(1, 0.0, 0.0, 0);
PIDController rollRateController  (0.224, 0.7923, 0.0019, 0);
PIDController pitchRateController (0.224, 0.7923, 0.0019, 0);
PIDController yawRateController   (0.2689, 0.5704, 0.0038, 0);
PIDController altitudeController  (0.1, 0.1, 0.1, 0);

// ============================================================
//  Helper functions
// ============================================================

float calSlope(float y2, float y1, float x2, float x1)
{
    return (float)(y2 - y1) / (x2 - x1);
}

float calYIntercept(float slope, float x, float y)
{
    return y - (slope * x);
}

// ============================================================
//  Update — called every loop iteration
// ============================================================

void update()
{
    currentTime = micros();
    deltaTime   = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    updateIMUReadings();
    // Serial.printf("pitch: %.2f, roll: %.2f, yaw: %.2f\n", pitch, roll, yaw);
    // Serial.printf("pitch: %.2f, roll: %.2f ", pitch, roll);
    // Serial.printf("%.2f,%.2f,%.2f\n", pitch, roll, yaw);
    updateAltitudeReadings();
    if (currentTime > printCount * 1000000) {
        Serial.printf("%i: ", printCount);
        Serial.printf("Altitude: %.2f\n", altitude);
        printCount++;
    }
    getChannelPacket();
    sendAltitudePacket();
    sendAttitudePacket();
    sendBatteryPacket();


    // Only handle web clients when upper-right switch is up
    // if (channels.ch[7].data > 1200)
    // {
        static uint32_t lastWebHandle = 0;
        uint32_t now = micros();
        if (now - lastWebHandle > 10000)
        {
            server.handleClient();
            lastWebHandle = now;
        }
    // }
}

// ============================================================
//  Setup
// ============================================================

void setup()
{
    Serial.begin(460800);
    Serial1.begin(bRate, SERIAL_8N1, Rx, Tx);
    delay(2000);

    // Serial.printf("Started Serial1 at %i baud\n", bRate);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, i2cSpeed);
    delay(100);

    initBMP388();
    delay(100);

    getInitialPressure(InitSampleSize);
    delay(100);

    ICM45686_Init();

    // channels.setDutyCycle(PWMFrequency, PWMResolution);

    ledcSetup(frontLeft,  PWMFrequency, PWMResolution);
    ledcSetup(frontRight, PWMFrequency, PWMResolution);
    ledcSetup(rearRight,  PWMFrequency, PWMResolution);
    ledcSetup(rearLeft,   PWMFrequency, PWMResolution);

    ledcAttachPin(frontLeftPin,  frontLeft);
    ledcAttachPin(frontRightPin, frontRight);
    ledcAttachPin(rearRightPin,  rearRight);
    ledcAttachPin(rearLeftPin,   rearLeft);

    ledcWrite(frontLeft,  0);
    ledcWrite(frontRight, 0);
    ledcWrite(rearRight,  0);
    ledcWrite(rearLeft,   0);

    analogReadResolution(12);
    analogSetPinAttenuation(batteryADCPin, ADC_11db);

    PIDWebPage();
    // Serial.println("EXITING SETUP");
    previousTime = micros();
}

// ============================================================
//  Main loop
// ============================================================

void loop()
{
    update();

    // --------------------------------------------------------
    //  ARMED
    // --------------------------------------------------------
    if (channels.ch[4].data < 900)
    {
        Serial.println("armed");
        while (true)
        {
            update();
            if (channels.ch[4].data > 900) break;  // disarmed

            movement();

            // Lower right switch position 1 (low) — direct throttle pass-through
            if (channels.ch[6].data < 900)
            {
                ledcWrite(frontLeft,  mainThrottleInput);
                ledcWrite(frontRight, mainThrottleInput);
                ledcWrite(rearRight,  mainThrottleInput);
                ledcWrite(rearLeft,   mainThrottleInput);
            }
        }
    }
    // --------------------------------------------------------
    //  DISARMED
    // --------------------------------------------------------
    else if (channels.ch[4].data > 900)
    {
        Serial.println("disarmed");
        while (true)
        {
            update();
            if (channels.ch[4].data < 900) break;  // armed

            // Lower right switch position 1 (low) — ESC calibration
            if (channels.ch[6].data < 900)
            {
                if (mainThrottleInput == 2047)
                {
                    ledcWrite(frontLeft,  mainThrottleInput);
                    ledcWrite(frontRight, mainThrottleInput);
                    ledcWrite(rearRight,  mainThrottleInput);
                    ledcWrite(rearLeft,   mainThrottleInput);
                }
                else if (mainThrottleInput == 1023)
                {
                    ledcWrite(frontLeft,  mainThrottleInput);
                    ledcWrite(frontRight, mainThrottleInput);
                    ledcWrite(rearRight,  mainThrottleInput);
                    ledcWrite(rearLeft,   mainThrottleInput);
                }
            }
        }
    }
}
