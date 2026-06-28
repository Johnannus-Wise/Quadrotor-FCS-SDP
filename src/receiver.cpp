#include "receiver.h"
#include "globals.h"
// ============================================================
//  Channels — constructor & methods
// ============================================================

Channels::Channels(int angleMin, int angleMax,
                   int yawMin,   int yawMax,
                   int altitudeMin, int altitudeMax)
{
    angleSlope        = (float)(angleMax    - angleMin)    / (SIGNAL_MAX - SIGNAL_MIN);
    angleIntercept    = (float) angleMin    - (angleSlope     * SIGNAL_MIN);
    yawSlope          = (float)(yawMax      - yawMin)      / (SIGNAL_MAX - SIGNAL_MIN);
    yawIntercept      = (float) yawMin      - (yawSlope       * SIGNAL_MIN);
    altitudeSlope     = (float)(altitudeMax - altitudeMin) / (SIGNAL_MAX - SIGNAL_MIN);
    altitudeIntercept = (float) altitudeMin - (altitudeSlope  * SIGNAL_MIN);
    setDutyCycle(PWM_FREQUENCY, PWM_RESOLUTION);
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
    // displayReadings();
    rescale();
}

void Channels::rescale()
{
    mainThrottleInput = (int)(throttleSlope * (float)ch[2].data + throttleIntercept);
    
    desiredRoll  = (int)(angleSlope * (float)ch[0].data + angleIntercept);
    desiredPitch = -((int)(angleSlope * (float)ch[1].data + angleIntercept));

    yawSpeed       = (int)(yawSlope      * (float)ch[3].data + yawIntercept);
    desiredAltitude = (int)(altitudeSlope * (float)ch[2].data + altitudeIntercept);

    // Serial.printf("Rescaled: Throttle %i, Desired Roll %i°, Desired Pitch %i°, Yaw Speed %i°/s, Desired Altitude %i m\n",
    //               mainThrottleInput, desiredRoll, desiredPitch, yawSpeed, desiredAltitude);
    // Serial.printf("angleSlope: %.3f, angleIntercept: %.3f, yawSlope: %.3f, yawIntercept: %.2f, altitudeSlope: %.3f, altitudeIntercept: %.2f\n",
    //               angleSlope, angleIntercept, yawSlope, yawIntercept, altitudeSlope, altitudeIntercept);
}

void Channels::setDutyCycle(int desiredFrequency, int desiredResolution)
{
    if      (desiredFrequency > 500) desiredFrequency = 500;
    else if (desiredFrequency < 50)  desiredFrequency = 50;

    float period            = 1.0f / (float)desiredFrequency;
    int   range             = (1 << desiredResolution);   // BUG FIX: pow() → bit shift
    float minDuty           = 0.001f / period;
    float maxDuty           = 0.002f / period;
    MIN_THROTTLE_VALUE = (float)range * minDuty - 1;
    MAX_THROTTLE_VALUE = (float)range * maxDuty - 1;

    throttleSlope     = (MAX_THROTTLE_VALUE - MIN_THROTTLE_VALUE) / (float)(SIGNAL_MAX - SIGNAL_MIN);
    throttleIntercept = MIN_THROTTLE_VALUE - (throttleSlope * (float)SIGNAL_MIN);
}

void Channels::displayReadings()
{
    Serial.printf("ch0: %i ch1: %i ch2: %i ch3: %i ",
                  ch[0].data, ch[1].data, ch[2].data, ch[3].data);
    Serial.printf("ch4: %i ch5: %i ch6: %i ch7: %i ",
                  ch[4].data, ch[5].data, ch[6].data, ch[7].data);
    Serial.printf("ch8: %i ch9: %i ch10: %i ch11: %i ",
                  ch[8].data, ch[9].data, ch[10].data, ch[11].data);
    Serial.printf("ch12: %i ch13: %i ch14: %i ch15: %i\n",
                  ch[12].data, ch[13].data, ch[14].data, ch[15].data);
}

// Global Channels instance
Channels channels(-TILT_ANGLE_MAX , TILT_ANGLE_MAX,
                  -YAW_SPEED_MAX,       YAW_SPEED_MAX,
                  MIN_ALTITUDE,         MAX_ALTITUDE);

// ============================================================
//  CRC-8 / D5
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

// Static state machine for non-blocking packet reception
static struct {
    uint8_t  state;        // 0: waiting for sync, 1: reading packet
    uint8_t  idx;          // index into channelsPacket
    uint32_t lastByteTime; // micros() of last successful byte read
} rxState = {0, 0, 0};

#define RX_TIMEOUT_US 3000  // 3ms timeout for full packet (handles slow serial + jitter)


// Reads bytes from Serial1 and assembles CRSF channel packets.  Non-blocking and robust to desync.
// After setting the channel readings, it calls sendTelemetryPackets() to transmit telemetry back to the transmitter (round-robin).
void getChannelPacket()
{
    while (Serial1.available())
    {
        uint8_t byte = Serial1.read();
        uint32_t now = micros();
        
        // State 0: Waiting for CRSF_ADDR sync byte
        if (rxState.state == 0)
        {
            if (byte == CRSF_ADDR)
            {
                rxState.state = 1;
                rxState.idx = 0;
                channelsPacket[rxState.idx++] = byte;
                rxState.lastByteTime = now;
            }
        }
        // State 1: Receiving packet bytes
        else if (rxState.state == 1)
        {
            channelsPacket[rxState.idx++] = byte;
            rxState.lastByteTime = now;
            
            // After reading length byte, validate it
            if (rxState.idx == 2)
            {
                if (channelsPacket[1] != CRSF_LENGTH_CHANNELS)
                {
                    // Invalid length, resync
                    rxState.state = 0;
                    rxState.idx = 0;
                }
            }
            // After reading type byte, validate it
            else if (rxState.idx == 3)
            {
                if (channelsPacket[2] != CRSF_TYPE_CHANNELS)
                {
                    // Invalid type, resync
                    rxState.state = 0;
                    rxState.idx = 0;
                }
            }
            // Check if we have a complete packet (26 bytes total)
            else if (rxState.idx >= 26)
            {
                uint8_t computed = crc8_d5(channelsPacket + 2, channelsPacket[1] - 1);
                if (computed == channelsPacket[25])
                {
                    crc8_last = computed;
                    channels.setChannelReadings(channelsPacket + 3);
                }
                // Reset for next packet regardless of CRC result
                rxState.state = 0;
                rxState.idx = 0;
            }
        }
    }
    
    // Timeout: reset if no bytes received for too long
    if (rxState.state == 1)
    {
        uint32_t now = micros();
        if (now - rxState.lastByteTime > RX_TIMEOUT_US)
        {
            rxState.state = 0;
            rxState.idx = 0;
        }
    }
    sendTelemetryPackets();
}

// ============================================================
//  Telemetry packets → transmitter
// ============================================================

void sendAltitudePacket()
{
    uint8_t  pkt[8];
    uint16_t dm_alt = (uint16_t)((altitude * 10.0f) + 10000.0f);

    pkt[0] = CRSF_SENSOR;
    pkt[1] = 0x06;
    pkt[2] = CRSF_TYPE_ALTITUDE;
    pkt[3] = (uint8_t)(dm_alt >> 8);
    pkt[4] = (uint8_t)(dm_alt);
    pkt[5] = 0;
    pkt[6] = 0;
    pkt[7] = crc8_d5(pkt + 2, 5);

    Serial1.write(pkt, 8);
}

void sendAttitudePacket()
{
    uint8_t  pkt[10];
    // BUG FIX: original cast pitch/roll/yaw directly to uint16_t after
    // multiplying by 10000.  Negative angles produce negative int values
    // which truncate incorrectly when stored in uint16_t.  Cast to int16_t
    // first to preserve sign, then let the byte extraction handle it.
    int16_t pitchPkt = (int16_t)(((float)pitch * (float)PI / 180.0f) * 10000.0f);
    int16_t rollPkt  = (int16_t)(((float)roll  * (float)PI / 180.0f) * 10000.0f);
    int16_t yawPkt   = (int16_t)(((float)yaw   * (float)PI / 180.0f) * 10000.0f);

    pkt[0] = CRSF_SENSOR;
    pkt[1] = 0x08;
    pkt[2] = CRSF_TYPE_ATTITUDE;
    pkt[3] = (uint8_t)((uint16_t)pitchPkt >> 8);
    pkt[4] = (uint8_t)((uint16_t)pitchPkt);
    pkt[5] = (uint8_t)((uint16_t)rollPkt  >> 8);
    pkt[6] = (uint8_t)((uint16_t)rollPkt);
    pkt[7] = (uint8_t)((uint16_t)yawPkt   >> 8);
    pkt[8] = (uint8_t)((uint16_t)yawPkt);
    pkt[9] = crc8_d5(pkt + 2, 7);

    Serial1.write(pkt, 10);
}

void getBatteryValues()
{
    rawBatteryReadings = analogReadMilliVolts(BATTERY_ADC_PIN);

    const float ADC_DIODE_DROP = 0.928f;    //Calibrated in lab with a multimeter, accounts for the voltage drop across the diode in the voltage divider
    measuredVoltage = ((float)rawBatteryReadings / 4095.0f) * V_REF + ADC_DIODE_DROP;
    batteryVoltage  = measuredVoltage * RESISTANCE_RATIO;

    const float VOLTAGE_SWING = MAX_BATTERY_VOLTAGE - 12.0f;   // 4S: 16.8 - 12.0
    batteryPercentUsed        = (MAX_BATTERY_VOLTAGE - batteryVoltage) / VOLTAGE_SWING;
    batteryPercentUsed        = constrain(batteryPercentUsed, 0.0f, 1.0f);

    batteryCapacityRemaining  = (uint32_t)((1.0f - batteryPercentUsed) * (float)MAX_BATTERY_CAPACITY);
    batteryRemainingPercent   = (uint8_t)((1.0f  - batteryPercentUsed) * 100.0f);
}

void sendBatteryPacket()
{
    getBatteryValues();

    uint8_t  pkt[12];
    uint16_t voltage  = (uint16_t)(batteryVoltage * 10.0f);
    uint16_t current  = 10;  // placeholder — no current sensor fitted

    uint32_t batteryCurrentCapacity = (uint32_t)(batteryCapacityRemaining);
    uint32_t cap_pct = (batteryCurrentCapacity << 8) | (uint32_t)batteryRemainingPercent;

    pkt[0]  = CRSF_SENSOR;
    pkt[1]  = 0x0A;
    pkt[2]  = CRSF_TYPE_BATTERY;
    pkt[3]  = (uint8_t)(voltage >> 8);
    pkt[4]  = (uint8_t)(voltage);
    pkt[5]  = (uint8_t)(current >> 8);
    pkt[6]  = (uint8_t)(current);
    pkt[7]  = (uint8_t)(cap_pct >> 24);
    pkt[8]  = (uint8_t)(cap_pct >> 16);
    pkt[9]  = (uint8_t)(cap_pct >> 8);
    pkt[10] = (uint8_t)(cap_pct);
    pkt[11] = crc8_d5(pkt + 2, 9);

    Serial1.write(pkt, 12);
}

void sendTelemetryPackets()
{
    if (telemetrySelector == 0) {
        sendAttitudePacket();
        telemetrySelector = 1;
    }
    else if (telemetrySelector == 1) {
        sendAltitudePacket();
        telemetrySelector = 2;
    }
    else if (telemetrySelector == 2) {
        sendBatteryPacket();
        telemetrySelector = 0;
    }
}