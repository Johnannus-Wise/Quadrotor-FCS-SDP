#pragma once
#include "config.h"

// ============================================================
//  Bit11 / Channels structs
// ============================================================

struct Bit11
{
    unsigned data : 11;
};

struct Channels
{
    Bit11 ch[16];

    float throttleSlope,    throttleIntercept;
    float angleSlope,       angleIntercept;
    float yawSlope,         yawIntercept;
    float altitudeSlope,    altitudeIntercept;

    Channels(int angleMin, int angleMax,
             int yawMin,   int yawMax,
             int altitudeMin, int altitudeMax);

    void setChannelReadings(uint8_t refData[22]);
    void rescale();
    void setDutyCycle(int desiredFrequency, int desiredResolution);
    void displayReadings();
};

// Global channels instance — defined in receiver.cpp
extern Channels channels;

// ============================================================
//  Functions
// ============================================================
void    getChannelPacket();
void    sendAltitudePacket();
void    sendAttitudePacket();
void    sendBatteryPacket();
void    getBatteryValues();
uint8_t crc8_d5(const uint8_t *ptr, uint8_t len);
