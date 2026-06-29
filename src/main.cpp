#include <Wire.h>

#include "globals.h"
#include "bmp388.h"
#include "icm45686.h"
#include "receiver.h"
#include "motors.h"
#include "wifi_server.h"
#include "utils.h"

// ============================================================
//  update — sensor reads and comms, called every loop iteration
//
//    IMU  (Mahony)   : 3.2 kHz
//    Barometer read  : 50 Hz
//    CRSF rx + tx    : 250 Hz
//    Web server      : 10 Hz
// ============================================================
static void update()
{
    
    currentTime = micros();

    deltaTime    = (currentTime - previousTime) / 1000000.0f;
    previousTime = currentTime;
    if (currentTime - imuUpdateLast >= IMU_UPDATE_US)
    {
        imuUpdateLast = currentTime;
        updateIMUReadings();
        // Serial.printf("IMU Update — Pitch: %.2f°, Roll: %.2f°\n", pitch, roll);
    }

    updateAltitudeReadings();

    // ── CRSF receive + telemetry transmit ─────────────────────────
    if (currentTime - packetUpdateLast >= PACKET_UPDATE_US)
    {
        packetUpdateLast = currentTime;
        getChannelPacket();
    }

    // ── Web server — 10 Hz ─────────────────────────────────────────────────
    if (currentTime - webHandleLast >= WEB_HANDLE_US)
    {
        webHandleLast = currentTime;
        server.handleClient();
    }

    if (currentTime - flightUpdateLast >= FLIGHT_UPDATE_US)
    {
        flightUpdateLast = currentTime;
        flightLog();
    }
}

// ============================================================
//  setup
// ============================================================

void setup()
{
    Serial.begin(SerialBaudRate);
    Serial1.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    delay(2000);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_SPEED);
    delay(100);

    initBMP388();
    delay(100);

    getInitialPressure(INIT_SAMPLE_SIZE);
    delay(100);

    ICM45686_Init();

    ledcSetup(FRONT_LEFT,  PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(FRONT_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(REAR_RIGHT,  PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(REAR_LEFT,   PWM_FREQUENCY, PWM_RESOLUTION);

    // ledcAttachPin(FRONT_LEFT_PIN,  FRONT_LEFT);
    ledcAttachPin(FRONT_LEFT_LED,  FRONT_LEFT);

    // ledcAttachPin(FRONT_RIGHT_PIN, FRONT_RIGHT);
    ledcAttachPin(FRONT_RIGHT_LED, FRONT_RIGHT);

    // ledcAttachPin(REAR_RIGHT_PIN,  REAR_RIGHT);
    ledcAttachPin(REAR_RIGHT_LED,  REAR_RIGHT);

    // ledcAttachPin(REAR_LEFT_PIN,   REAR_LEFT);
    ledcAttachPin(REAR_LEFT_LED,   REAR_LEFT);

    ledcWrite(FRONT_LEFT,  0);
    ledcWrite(FRONT_RIGHT, 0);
    ledcWrite(REAR_RIGHT,  0);
    ledcWrite(REAR_LEFT,   0);

    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

    PIDWebPage();

    // Seed all timestamps so no timer fires immediately on first tick
    currentTime            = micros();
    attitudeLoopLast   = currentTime;
    altitudeLoopLast   = currentTime;
    packetUpdateLast   = currentTime;
    webHandleLast      = currentTime;
    imuUpdateLast        = currentTime;
    previousTime        = currentTime;
    Serial.printf("ATTITUDE_UPDATE_RATE: %lu\n", ATTITUDE_UPDATE_RATE);
    Serial.printf("ALTITUDE_UPDATE_RATE: %lu\n", ALTITUDE_UPDATE_RATE);
    Serial.printf("PACKET_UPDATE_RATE: %lu\n", PACKET_UPDATE_RATE);
    Serial.printf("WEB_HANDLE_UPDATE_RATE: %lu\n", WEB_HANDLE_UPDATE_RATE);
    Serial.printf("IMU_UPDATE_RATE: %lu\n", IMU_UPDATE_RATE);
    update();
}

// ============================================================
//  loop
// ============================================================

void loop()
{
    // --------------------------------------------------------
    //  ARMED  (ch[4] low)
    // --------------------------------------------------------
    if (channels.ch[4].data < 900)
    {
        Serial.println("ARMED");
        resetControllers(); //Executed once on arming.
        // Renew the reference pressure baseline when armed
        // This establishes current altitude as 0 m for this flight
        // renewReferencePressure(INIT_SAMPLE_SIZE);
        while (true)
        {
            update();
            if (channels.ch[4].data > 900) break;   // disarm

            // ch[6] low → direct throttle pass-through (test / spin-up)
            if (channels.ch[6].data < 900)
            {
                // Serial.println("Direct Throttle Mode");
                ledcWrite(FRONT_LEFT,  mainThrottleInput);
                ledcWrite(FRONT_RIGHT, mainThrottleInput);
                ledcWrite(REAR_RIGHT,  mainThrottleInput);
                ledcWrite(REAR_LEFT,   mainThrottleInput);
            }
            else if (channels.ch[6].data > 900)
            {
                // Serial.println("Flight Control Mode");
                movement();
            }
        }
        // Reset all PID controllers when transitioning to disarmed
        // rollAngleController.reset();
        // pitchAngleController.reset();
        // rollRateController.reset();
        // pitchRateController.reset();
        // yawRateController.reset();
        // altitudeController.reset();
    }
    // --------------------------------------------------------
    //  DISARMED  (ch[4] high)
    // --------------------------------------------------------
    else
    {
        Serial.println("DISARMED");
        while (true)
        {
            update();
            if (channels.ch[4].data < 900) break;   // arm

            // ch[6] low → ESC calibration sequence
            if (channels.ch[6].data < 900)
            {
                // Serial.println("ESC Calibration Mode");
                // channels.displayReadings();
                if (mainThrottleInput == MAX_THROTTLE_VALUE)
                {
                    // Serial.println("ESC Calibration: Max Throttle");
                    ledcWrite(FRONT_LEFT,  mainThrottleInput);
                    ledcWrite(FRONT_RIGHT, mainThrottleInput);
                    ledcWrite(REAR_RIGHT,  mainThrottleInput);
                    ledcWrite(REAR_LEFT,   mainThrottleInput);
                }
                else if (mainThrottleInput == MIN_THROTTLE_VALUE)
                {
                    // Serial.println("ESC Calibration: Min Throttle");
                    ledcWrite(FRONT_LEFT,  mainThrottleInput);
                    ledcWrite(FRONT_RIGHT, mainThrottleInput);
                    ledcWrite(REAR_RIGHT,  mainThrottleInput);
                    ledcWrite(REAR_LEFT,   mainThrottleInput);
                }
            }
        }
    }
}
