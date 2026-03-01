#include "receiver.h"

// ============================================================
//  Channels — constructor & methods
// ============================================================

Channels::Channels(int angleMin, int angleMax,
                   int yawMin,   int yawMax,
                   int altitudeMin, int altitudeMax)
{
    angleSlope       = (float)(angleMax - angleMin)       / (signalMax - signalMin);
    angleIntercept   = (float)(angleMin - (angleSlope * signalMin));
    yawSlope         = (float)(yawMax - yawMin)           / (signalMax - signalMin);
    yawIntercept     = (float)(yawMin - (yawSlope * signalMin));
    altitudeSlope    = (float)(altitudeMax - altitudeMin) / (signalMax - signalMin);
    altitudeIntercept = (float)(altitudeMin - (altitudeSlope * signalMin));
}

void Channels::setChannelReadings(uint8_t refData[22])
{
    ch[0].data  = refData[0]  | (refData[1]  << 8);
    ch[1].data  = (refData[1]  >> 3) | (refData[2]  << 5);
    ch[2].data  = ((refData[2]  >> 6) | (refData[3]  << 2) | (refData[4]  << 10));
    ch[3].data  = (refData[4]  >> 1) | (refData[5]  << 7);
    ch[4].data  = (refData[5]  >> 4) | (refData[6]  << 4);
    ch[5].data  = (refData[6]  >> 7) | (refData[7]  << 1) | (refData[8]  << 9);
    ch[6].data  = (refData[8]  >> 2) | (refData[9]  << 6);
    ch[7].data  = (refData[9]  >> 5) | (refData[10] << 3);
    ch[8].data  = (refData[11])      | (refData[12] << 8);
    ch[9].data  = (refData[12] >> 3) | (refData[13] << 5);
    ch[10].data = (refData[13] >> 6) | (refData[14] << 2) | (refData[15] << 10);
    ch[11].data = (refData[15] >> 1) | (refData[16] << 7);
    ch[12].data = (refData[16] >> 4) | (refData[17] << 4);
    ch[13].data = (refData[17] >> 7) | (refData[18] << 1) | (refData[19] << 9);
    ch[14].data = (refData[19] >> 2) | (refData[20] << 6);
    ch[15].data = (refData[20] >> 5) | (refData[21] << 3);
    rescale();
}

void Channels::rescale()
{
    mainThrottleInput = (int)(throttleSlope * ch[2].data + throttleIntercept);

    if (mainThrottleInput > 0.6 * maxThrottleValue)
    {
        desiredRoll  = angleSlope * ch[0].data + angleIntercept;
        desiredPitch = angleSlope * ch[1].data + angleIntercept;
    }
    else
    {
        desiredRoll  = 0;
        desiredPitch = 0;
    }
    yawSpeed       = yawSlope      * ch[3].data + yawIntercept;
    desiredAltitude = altitudeSlope * ch[2].data + altitudeIntercept;
}

void Channels::setDutyCycle(int desiredFrequency, int desiredResolution)
{
    if      (desiredFrequency > 500) desiredFrequency = 500;
    else if (desiredFrequency < 50)  desiredFrequency = 50;

    float period            = 1.0 / desiredFrequency;
    int   range             = pow(2.0, desiredResolution);
    float minDuty           = 0.001 / period;
    float maxDuty           = 0.002 / period;
    float minThrottleSignal = range * minDuty;
    float maxThrottleSignal = range * maxDuty;

    throttleSlope     = (float)(maxThrottleSignal - minThrottleSignal) / (signalMax - signalMin);
    throttleIntercept = (float)(minThrottleSignal - (throttleSlope * signalMin));
}

void Channels::displayReadings()
{
    Serial.printf("ch0: %i ch1: %i, ch2: %i, ch3: %i ",  desiredRoll, desiredPitch, ch[2].data, yawSpeed);
    Serial.printf("ch4: %i ch5: %i, ch6: %i, ch7: %i ",  ch[4].data, ch[5].data, ch[6].data, ch[7].data);
    Serial.printf("ch8: %i ch9: %i, ch10: %i, ch11: %i ", ch[8].data, ch[9].data, ch[10].data, ch[11].data);
    Serial.printf("ch12: %i ch13: %i, ch14: %i, ch15: %i\n", ch[12].data, ch[13].data, ch[14].data, ch[15].data);
}

// Global instance
Channels channels(-tiltAngleMax - 1, tiltAngleMax, -yawSpeedMax, yawSpeedMax, minAltitude, maxAltitude);

// ============================================================
//  CRC8
// ============================================================

uint8_t crc8_d5(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        uint8_t inbyte = *ptr++;
        for (uint8_t i = 0; i < 8; i++)
        {
            uint8_t mix = (crc ^ inbyte) & 0x80;
            crc <<= 1;
            if (mix) crc ^= 0xD5;
            inbyte <<= 1;
        }
    }
    return crc;
}

// ============================================================
//  Packet receive
// ============================================================

void getChannelPacket()
{
    uint8_t lastRead = Serial1.read();
    if (lastRead == CRSF_Addr)
    {
        channelsPacket[0] = lastRead;
        lastRead          = Serial1.read();
        if (lastRead == CRSF_Length_Channels)
        {
            channelsPacket[1] = lastRead;
            lastRead          = Serial1.read();
            if (lastRead == CRSF_Type_Channels)
            {
                channelsPacket[2] = lastRead;
                for (int i = 0; i < channelsPacket[1] - 1; i++)
                {
                    channelsPacket[i + 3] = Serial1.read();
                }
                crc8 = crc8_d5(channelsPacket + 2, channelsPacket[1] - 1);
                if (crc8 == channelsPacket[25])
                {
                    channels.setChannelReadings(channelsPacket + 3);
                }
            }
        }
    }
}

// ============================================================
//  Telemetry packets sent back to transmitter
// ============================================================

void sendAltitudePacket()
{
    uint8_t  altitudePacket[8];
    uint16_t dm_Altitude = (altitude * 10) + 10000;

    altitudePacket[0] = CRSF_Sensor;
    altitudePacket[1] = 0x06;
    altitudePacket[2] = CRSF_Type_Altitude;
    altitudePacket[3] = (dm_Altitude >> 8);
    altitudePacket[4] = dm_Altitude;
    altitudePacket[5] = 0;
    altitudePacket[6] = 0;
    altitudePacket[7] = crc8_d5(altitudePacket + 2, 5);

    Serial1.write(altitudePacket, 8);
}

void sendAttitudePacket()
{
    uint8_t  attitudePacket[10];
    uint16_t pitchPacket = ((pitch * PI) / 180) * 10000;
    uint16_t rollPacket  = ((roll  * PI) / 180) * 10000;
    uint16_t yawPacket   = ((yaw   * PI) / 180) * 10000;

    attitudePacket[0] = CRSF_Sensor;
    attitudePacket[1] = 0x08;
    attitudePacket[2] = CRSF_Type_Attitude;
    attitudePacket[3] = pitchPacket >> 8;
    attitudePacket[4] = pitchPacket;
    attitudePacket[5] = rollPacket  >> 8;
    attitudePacket[6] = rollPacket;
    attitudePacket[7] = yawPacket   >> 8;
    attitudePacket[8] = yawPacket;
    attitudePacket[9] = crc8_d5(attitudePacket + 2, 7);

    Serial1.write(attitudePacket, 10);
}

void getBatteryValues()
{
    rawBatteryReadings       = analogRead(batteryADCPin);
    measuredVoltage          = (rawBatteryReadings / 4095.0) * Vref + 0.4;
    batteryVoltage           = measuredVoltage * resistanceRatio;
    batteryPercentUsed       = ((16.8 - batteryVoltage) / 4.8);
    batteryCapacityRemaining = (1 - batteryPercentUsed) * maxBatteryCapacity;
    batteryRemainingPercent  = (1 - batteryPercentUsed) * 100;
}

void sendBatteryPacket()
{
    getBatteryValues();
    uint8_t  batteryPacket[12];
    uint16_t voltage    = batteryVoltage * 10;
    uint16_t current    = 10;
    uint32_t usedCapacity_remaining = (batteryCapacityRemaining << 8) | batteryRemainingPercent;

    batteryPacket[0]  = CRSF_Sensor;
    batteryPacket[1]  = 0x0A;
    batteryPacket[2]  = CRSF_Type_Battery;
    batteryPacket[3]  = voltage >> 8;
    batteryPacket[4]  = voltage;
    batteryPacket[5]  = current >> 8;
    batteryPacket[6]  = current;
    batteryPacket[7]  = usedCapacity_remaining >> 24;
    batteryPacket[8]  = usedCapacity_remaining >> 16;
    batteryPacket[9]  = usedCapacity_remaining >> 8;
    batteryPacket[10] = usedCapacity_remaining;
    batteryPacket[11] = crc8_d5(batteryPacket + 2, 9);

    Serial1.write(batteryPacket, 12);
}
