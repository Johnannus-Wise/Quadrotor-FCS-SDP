#pragma once
#include <Arduino.h>

// ===== Wi-Fi credentials =====
extern const char *ssid;
extern const char *password;

// ===== BMP388 =====
#define BMP388ADDRESS   0x76
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_SPEED       400000      // BUG FIX: was 1000000 (1 MHz). The BMP388
                                    // I²C fast-mode spec is 400 kHz. 1 MHz is
                                    // "fast-mode plus" and not guaranteed on all
                                    // ESP32 silicon / pull-up combinations.
                                    // 400 kHz is safe and still leaves IMU headroom.
#define P0              101325
#define INIT_SAMPLE_SIZE 200

// ===== ICM-45686 =====
#define ICM_ADDRESS         0x68
#define ACCEL_SENSITIVITY   8192.0f // BUG FIX: was plain int; used in float division
#define GYRO_SENSITIVITY    65.536f // BUG FIX: same — use float literal

// ===== CRSF / Receiver =====
#define CRSF_ADDR               0xC8
#define CRSF_SENSOR             0xEA
#define CRSF_TYPE_BATTERY       0x08
#define CRSF_TYPE_ALTITUDE      0x09
#define CRSF_TYPE_CHANNELS      0x16
#define CRSF_TYPE_ATTITUDE      0x1E
#define CRSF_LENGTH_CHANNELS    0x18
#define CRSF_LENGTH_ALTITUDE    0x06
#define CRSF_RX_PIN             16
#define CRSF_TX_PIN             17
#define CRSF_BAUD               416666

// ===== Motor / PWM =====
#define FRONT_LEFT_PIN  32  //M1 Green Wire
#define FRONT_RIGHT_PIN 25  //M2 Red Wire
#define REAR_RIGHT_PIN  26  //M3 Orange Wire
#define REAR_LEFT_PIN   33  //M4 Yellow Wire
#define FRONT_LEFT      1
#define FRONT_RIGHT     2
#define REAR_RIGHT      3
#define REAR_LEFT       4

// ===== PWM / signal =====
#define PWM_FREQUENCY   500
#define PWM_RESOLUTION  11
#define PWM_RANGE       ((int)(1 << PWM_RESOLUTION))
#define SIGNAL_MAX      1811
#define SIGNAL_MIN      174
#define MIN_THROTTLE_FRAC   0.525f   // 5 % throttle floor
#define MAX_THROTTLE_FRAC   0.90f    // 90 % throttle ceiling

// ===== Flight limits =====
#define TILT_ANGLE_MAX      25
#define YAW_SPEED_MAX       90
#define MIN_ALTITUDE        0
#define MAX_ALTITUDE        30
#define MAX_THROTTLE_VALUE  (PWM_RANGE - 1)
#define MIN_THROTTLE_VALUE  ((PWM_RANGE / 2) - 1)

// ===== Battery =====
#define BATTERY_ADC_PIN     36
#define R1                  10000.0f
#define R2                  1980.0f
#define RESISTANCE_RATIO    ((R1 + R2) / R2)
#define V_REF               2.8f
#define MAX_BATTERY_VOLTAGE 16.8f
#define MAX_BATTERY_CAPACITY 10400
