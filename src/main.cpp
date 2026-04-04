#include <Arduino.h>
#include <Wire.h>

#include "globals.h"
#include "bmp388.h"
#include "icm45686.h"
#include "receiver.h"
#include "motors.h"
#include "wifi_server.h"

// ============================================================
//  update — sensor reads, telemetry, web client poll
//  Called at the top of every loop iteration (armed and disarmed).
// ============================================================

static void update()
{
    currentTime  = (float)micros();
    deltaTime    = (currentTime - previousTime) / 1000000.0f;
    previousTime = currentTime;

    // Guard against runaway deltaTime on first tick or after a long pause
    if (deltaTime > 0.05f) deltaTime = 0.05f;   // cap at 50 ms (20 Hz minimum)

    updateIMUReadings();
    updateAltitudeReadings();
    getChannelPacket();
    sendAltitudePacket();
    sendAttitudePacket();
    sendBatteryPacket();

    // Service web clients at ~100 Hz (every 10 ms) without blocking the loop
    static uint32_t lastWebHandle = 0;
    uint32_t nowUs = (uint32_t)currentTime;
    if (nowUs - lastWebHandle > 10000)
    {
        server.handleClient();
        lastWebHandle = nowUs;
    }
}

// ============================================================
//  setup
// ============================================================

void setup()
{
    Serial.begin(460800);
    Serial1.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    delay(2000);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_SPEED);
    delay(100);

    initBMP388();
    delay(100);

    getInitialPressure(INIT_SAMPLE_SIZE);
    delay(100);

    ICM45686_Init();

    // Configure PWM channels
    ledcSetup(FRONT_LEFT,  PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(FRONT_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(REAR_RIGHT,  PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(REAR_LEFT,   PWM_FREQUENCY, PWM_RESOLUTION);

    ledcAttachPin(FRONT_LEFT_PIN,  FRONT_LEFT);
    ledcAttachPin(FRONT_RIGHT_PIN, FRONT_RIGHT);
    ledcAttachPin(REAR_RIGHT_PIN,  REAR_RIGHT);
    ledcAttachPin(REAR_LEFT_PIN,   REAR_LEFT);

    // Start motors at zero
    ledcWrite(FRONT_LEFT,  0);
    ledcWrite(FRONT_RIGHT, 0);
    ledcWrite(REAR_RIGHT,  0);
    ledcWrite(REAR_LEFT,   0);

    analogReadResolution(12);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);

    PIDWebPage();

    previousTime = (float)micros();
}

// ============================================================
//  loop
// ============================================================

void loop()
{
    update();

    // --------------------------------------------------------
    //  ARMED  (ch[4] low)
    // --------------------------------------------------------
    if (channels.ch[4].data < 900)
    {
        Serial.println("ARMED");
        while (true)
        {
            update();
            if (channels.ch[4].data > 900) break;   // disarm

            movement();

            // ch[6] low → direct throttle pass-through (test / spin-up)
            if (channels.ch[6].data < 900)
            {
                ledcWrite(FRONT_LEFT,  mainThrottleInput);
                ledcWrite(FRONT_RIGHT, mainThrottleInput);
                ledcWrite(REAR_RIGHT,  mainThrottleInput);
                ledcWrite(REAR_LEFT,   mainThrottleInput);
            }
        }
        // Reset all PID controllers when transitioning to disarmed
        rollAngleController.reset();
        pitchAngleController.reset();
        rollRateController.reset();
        pitchRateController.reset();
        yawRateController.reset();
        altitudeController.reset();
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
                if (mainThrottleInput == MAX_THROTTLE_VALUE)
                {
                    ledcWrite(FRONT_LEFT,  mainThrottleInput);
                    ledcWrite(FRONT_RIGHT, mainThrottleInput);
                    ledcWrite(REAR_RIGHT,  mainThrottleInput);
                    ledcWrite(REAR_LEFT,   mainThrottleInput);
                }
                else if (mainThrottleInput == MIN_THROTTLE_VALUE)
                {
                    ledcWrite(FRONT_LEFT,  mainThrottleInput);
                    ledcWrite(FRONT_RIGHT, mainThrottleInput);
                    ledcWrite(REAR_RIGHT,  mainThrottleInput);
                    ledcWrite(REAR_LEFT,   mainThrottleInput);
                }
            }
        }
    }
}
