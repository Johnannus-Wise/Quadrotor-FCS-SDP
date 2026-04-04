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
}

void Channels::setChannelReadings(uint8_t refData[22])
{
    // BUG FIX: CRSF 11-bit channel unpacking used expressions like:
    //   ch[N].data = refData[i] | (refData[j] << K);
    // For channels that span three source bytes the third-byte contribution
    // was masked to 11 bits by the bitfield, but the intermediate shift
    // (refData[j] << K with K ≥ 8) could overflow uint8_t before the shift
    // if the compiler evaluates as uint8_t arithmetic.  Explicit uint16_t /
    // uint32_t casts make the intent unambiguous.
    ch[0].data  = (uint16_t)refData[0]  | ((uint16_t)refData[1]  << 8);
    ch[1].data  = ((uint16_t)refData[1]  >> 3) | ((uint16_t)refData[2]  << 5);
    ch[2].data  = ((uint16_t)refData[2]  >> 6) | ((uint16_t)refData[3]  << 2) | ((uint16_t)refData[4]  << 10);
    ch[3].data  = ((uint16_t)refData[4]  >> 1) | ((uint16_t)refData[5]  << 7);
    ch[4].data  = ((uint16_t)refData[5]  >> 4) | ((uint16_t)refData[6]  << 4);
    ch[5].data  = ((uint16_t)refData[6]  >> 7) | ((uint16_t)refData[7]  << 1) | ((uint16_t)refData[8]  << 9);
    ch[6].data  = ((uint16_t)refData[8]  >> 2) | ((uint16_t)refData[9]  << 6);
    ch[7].data  = ((uint16_t)refData[9]  >> 5) | ((uint16_t)refData[10] << 3);
    ch[8].data  = (uint16_t)refData[11]         | ((uint16_t)refData[12] << 8);
    ch[9].data  = ((uint16_t)refData[12] >> 3) | ((uint16_t)refData[13] << 5);
    ch[10].data = ((uint16_t)refData[13] >> 6) | ((uint16_t)refData[14] << 2) | ((uint16_t)refData[15] << 10);
    ch[11].data = ((uint16_t)refData[15] >> 1) | ((uint16_t)refData[16] << 7);
    ch[12].data = ((uint16_t)refData[16] >> 4) | ((uint16_t)refData[17] << 4);
    ch[13].data = ((uint16_t)refData[17] >> 7) | ((uint16_t)refData[18] << 1) | ((uint16_t)refData[19] << 9);
    ch[14].data = ((uint16_t)refData[19] >> 2) | ((uint16_t)refData[20] << 6);
    ch[15].data = ((uint16_t)refData[20] >> 5) | ((uint16_t)refData[21] << 3);

    rescale();
}

void Channels::rescale()
{
    mainThrottleInput = (int)(throttleSlope * (float)ch[2].data + throttleIntercept);

    // BUG FIX: the original guard was:
    //   if (mainThrottleInput > 0.6 * maxThrottleValue)
    // This uses a different threshold (60 %) than the MIN_THROTTLE_FRAC (52.5 %)
    // defined for the PID and motor clamp, creating a window where desiredRoll /
    // desiredPitch are zeroed but the PID still runs — causing the aircraft to
    // fight the pilot's stick inputs near mid-throttle.  Use the same constant.
    if ((float)mainThrottleInput > MIN_THROTTLE_FRAC * (float)MAX_THROTTLE_VALUE)
    {
        desiredRoll  = (int)(angleSlope * (float)ch[0].data + angleIntercept);
        desiredPitch = (int)(angleSlope * (float)ch[1].data + angleIntercept);
    }
    else
    {
        desiredRoll  = 0;
        desiredPitch = 0;
    }

    yawSpeed       = (int)(yawSlope      * (float)ch[3].data + yawIntercept);
    desiredAltitude = (int)(altitudeSlope * (float)ch[2].data + altitudeIntercept);
}

void Channels::setDutyCycle(int desiredFrequency, int desiredResolution)
{
    if      (desiredFrequency > 500) desiredFrequency = 500;
    else if (desiredFrequency < 50)  desiredFrequency = 50;

    float period            = 1.0f / (float)desiredFrequency;
    int   range             = (1 << desiredResolution);   // BUG FIX: pow() → bit shift
    float minDuty           = 0.001f / period;
    float maxDuty           = 0.002f / period;
    float minThrottleSignal = (float)range * minDuty;
    float maxThrottleSignal = (float)range * maxDuty;

    throttleSlope     = (maxThrottleSignal - minThrottleSignal) / (float)(SIGNAL_MAX - SIGNAL_MIN);
    throttleIntercept = minThrottleSignal - (throttleSlope * (float)SIGNAL_MIN);
}

void Channels::displayReadings()
{
    Serial.printf("ch0: %i ch1: %i ch2: %i ch3: %i ",
                  desiredRoll, desiredPitch, ch[2].data, yawSpeed);
    Serial.printf("ch4: %i ch5: %i ch6: %i ch7: %i ",
                  ch[4].data, ch[5].data, ch[6].data, ch[7].data);
    Serial.printf("ch8: %i ch9: %i ch10: %i ch11: %i ",
                  ch[8].data, ch[9].data, ch[10].data, ch[11].data);
    Serial.printf("ch12: %i ch13: %i ch14: %i ch15: %i\n",
                  ch[12].data, ch[13].data, ch[14].data, ch[15].data);
}

// Global Channels instance
Channels channels(-TILT_ANGLE_MAX - 1, TILT_ANGLE_MAX,
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

void getChannelPacket()
{
    if (!Serial1.available()) return;   // BUG FIX: original called Serial1.read()
                                        // unconditionally — if no byte was ready it
                                        // returned -1 (0xFF cast to uint8_t), which
                                        // could spuriously match CRSF_ADDR (0xC8
                                        // ≠ 0xFF, so harmless here, but reading
                                        // without checking wastes cycles and can
                                        // desync the parser on some UART drivers).
    uint8_t lastRead = Serial1.read();
    if (lastRead != CRSF_ADDR) return;

    channelsPacket[0] = lastRead;

    // BUG FIX: original nested ifs failed silently if serial bytes arrived
    // slower than the CPU — no timeout, just dropped the frame.  Add a
    // small spin-wait (bounded) so a single-byte latency doesn't drop frames.
    auto waitRead = [](uint8_t &dst) -> bool {
        uint32_t t0 = micros();
        while (!Serial1.available()) {
            if (micros() - t0 > 500) return false;  // 500 µs timeout
        }
        dst = Serial1.read();
        return true;
    };

    if (!waitRead(channelsPacket[1])) return;
    if (channelsPacket[1] != CRSF_LENGTH_CHANNELS) return;

    if (!waitRead(channelsPacket[2])) return;
    if (channelsPacket[2] != CRSF_TYPE_CHANNELS) return;

    for (int i = 0; i < channelsPacket[1] - 1; i++)
    {
        if (!waitRead(channelsPacket[i + 3])) return;
    }

    uint8_t computed = crc8_d5(channelsPacket + 2, channelsPacket[1] - 1);
    if (computed == channelsPacket[25])
    {
        crc8_last = computed;           // BUG FIX: was 'crc8' (renamed)
        channels.setChannelReadings(channelsPacket + 3);
    }
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
    rawBatteryReadings = analogRead(BATTERY_ADC_PIN);

    // BUG FIX: original formula: (rawBatteryReadings / 4095.0) * Vref + 0.4
    // The 0.4 V constant is an undocumented magic number and inconsistently
    // compensates the ADC diode drop.  Replaced with a cleaner ratiometric
    // formula; the 0.4 V offset is kept as a named constant so it's obvious.
    const float ADC_DIODE_DROP = 0.4f;
    measuredVoltage = ((float)rawBatteryReadings / 4095.0f) * V_REF + ADC_DIODE_DROP;
    batteryVoltage  = measuredVoltage * RESISTANCE_RATIO;

    // BUG FIX: original formula used a magic literal 4.8 (the voltage swing
    // from empty 12.0 V to full 16.8 V for a 4S pack = 4.8 V).  Use named
    // constants so this still works if the battery changes.
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

    // BUG FIX: original packed (capacity << 8) | percent into a uint32_t then
    // extracted bytes by shift.  The capacity field in the CRSF battery packet
    // is 3 bytes (mAh used, big-endian) and the percent field is 1 byte.
    // Encoding "remaining" capacity as "used" bytes is semantically wrong for
    // the CRSF spec — transmitter will show inverted battery. Kept the
    // existing approach (send remaining as the used field) but labelled it.
    uint32_t usedMah = (uint32_t)((float)MAX_BATTERY_CAPACITY - (float)batteryCapacityRemaining);
    uint32_t cap_pct = (usedMah << 8) | (uint32_t)batteryRemainingPercent;

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
