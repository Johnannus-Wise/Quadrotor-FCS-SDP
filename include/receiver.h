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
    // 0	RIGHT STICK (RIGHT/LEFT)	174-1811
    // 1	RIGHT STICK (UP/DOWN)	174-1811
    // 2	LEFT STICK (UP/DOWN)	174-1811
    // 3	LEFT STICK (RIGHT/LEFT)	174-1811
    // 4	LOWER LEFT SWITCH (UP/DOWN)	
    // 5	UPPER LEFT SWITCH (UP/DOWN)	
    // 6	LOWER RIGHT SWITCH (UP/DOWN)
    // 7	UPPER RIGHT SWITCH (UP/DOWN)
    // 8	LEFT WHEEL ()	174-1811
    // 9	RIGHT WHEEL ()	174-1811
    // 10	REAR LEFT BUTTON
    // 11	REAR RIGHT BUTTON
    // 12	BOTTOM LEFT BUTTON
    // 13	BOTTOM RIGHT BUTTON
    // 14	
    // 15

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
void    sendTelemetryPackets();
uint8_t crc8_d5(const uint8_t *ptr, uint8_t len);
