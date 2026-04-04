// =============================================================================
//  bmp388.cpp — Enhanced BMP388 barometer driver
//
//  Signal chain (each step builds on the previous):
//    Raw 24-bit ADC
//      └─► Hardware IIR filter (coeff 7)      — kills high-freq noise in silicon
//            └─► Bosch compensated pressure (Pa)
//                  └─► Hypsometric altitude (m)
//                        └─► Median-of-3 spike rejection   — kills single-sample outliers
//                              └─► EMA stage 1 (α=0.80, ~2 Hz cutoff)   — fast smoothing
//                                    └─► EMA stage 2 (α=0.92, ~0.6 Hz cutoff) — final smooth
//                                          └─► Thermal drift correction
//                                                └─► Hysteresis gate (±0.05 m dead-band)
//                                                      └─► altitude (published to PID)
// =============================================================================

#include "bmp388.h"

// ---------------------------------------------------------------------------
//  Tuning constants
// ---------------------------------------------------------------------------

// Hardware IIR coefficient (reg 0x1F)
//   0x04 = coeff 3 (original) | 0x06 = coeff 7 (better) | 0x0E = coeff 127 (max)
#define BMP_IIR_REG_VALUE   0x06    // coeff 7 — good balance at 50 Hz ODR

// Oversampling (reg 0x1C): bits[2:0] = pressure, bits[5:3] = temp (shifted)
//   Pressure: 0x00=x1 … 0x05=x32
//   x32 pressure: needs ODR ≤ 50 Hz.  Temp x1 keeps measurement time minimal.
#define BMP_OSR_PRESSURE    0x05    // x32 pressure
#define BMP_OSR_TEMP        0x00    // x1 temp (no shift needed — bits 5:3 = 0)
#define BMP_OSR_REG_VALUE   (BMP_OSR_TEMP | BMP_OSR_PRESSURE)

// ODR register (0x1D)
//   0x00=200Hz … 0x02=50Hz … 0x03=25Hz
//   x32 pressure requires ≥25 Hz settling; 50 Hz gives headroom.
#define BMP_ODR_REG_VALUE   0x02    // 50 Hz

// Software EMA — two cascaded stages
//   alpha close to 1 → sluggish/smooth; close to 0 → fast/noisy
//   Stage 1 (fast): −3 dB at ~2 Hz   @ 50 Hz ODR
//   Stage 2 (slow): −3 dB at ~0.6 Hz @ 50 Hz ODR
#define EMA1_ALPHA          0.80f
#define EMA2_ALPHA          0.92f

// Hysteresis dead-band: only publish when filtered altitude moves > this amount
#define ALTITUDE_HYSTERESIS_M   0.05f   // metres — approx 1× noise floor

// Thermal drift gain (m/°C) — tune empirically
//   Typical BMP388 drift: ~0.08 m per °C of die temperature rise.
//   If altitude creeps up on warm-up, increase; if it drops, negate.
#define THERMAL_DRIFT_GAIN  0.08f

// ---------------------------------------------------------------------------
//  Module-private state
// ---------------------------------------------------------------------------

static float ema1         = 0.0f;
static float ema2         = 0.0f;
static float altitudeOut  = 0.0f;  // last published value (hysteresis reference)
static bool  filterSeeded = false;
static float prev1        = 0.0f;  // median-of-3 history
static float prev2        = 0.0f;

// ---------------------------------------------------------------------------
//  Internal helpers
// ---------------------------------------------------------------------------

// Median of 3 without sorting — exactly 3 comparisons, no temporaries
static inline float median3(float a, float b, float c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void initBMP388()
{
    // BMP388 requires sleep mode before any config register is changed
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1B);   // PWR_CTRL
    Wire.write(0x00);   // sleep, sensors on
    Wire.endTransmission();
    delay(20);

    // Oversampling: x32 pressure, x1 temperature
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1C);
    Wire.write(BMP_OSR_REG_VALUE);
    Wire.endTransmission();
    delay(20);

    // Output data rate: 50 Hz
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1D);
    Wire.write(BMP_ODR_REG_VALUE);
    Wire.endTransmission();
    delay(20);

    // Hardware IIR filter: coefficient 7
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1F);
    Wire.write(BMP_IIR_REG_VALUE);
    Wire.endTransmission();
    delay(20);

    // Normal mode: continuous measurement, both sensors enabled
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1B);
    Wire.write(0x33);
    Wire.endTransmission();
    delay(20);

    getCompensationData();
    delay(20);
}

void checkSensorStatus()
{
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP388ADDRESS, 1);
    uint8_t status = Wire.read();

    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1B);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP388ADDRESS, 1);
    uint8_t pwr_ctrl = Wire.read();

    Serial.printf("Sensor status = 0x%02X | PWR_CTRL = 0x%02X\n", status, pwr_ctrl);
}

void getCompensationData()
{
    int8_t   NVM_PAR_T3, NVM_PAR_P3, NVM_PAR_P4, NVM_PAR_P7,
             NVM_PAR_P8, NVM_PAR_P10, NVM_PAR_P11;
    int16_t  NVM_PAR_P1, NVM_PAR_P2, NVM_PAR_P9;
    uint16_t NVM_PAR_T1, NVM_PAR_T2, NVM_PAR_P5, NVM_PAR_P6;

    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x31);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP388ADDRESS, 21);
    delay(500);

    if (Wire.available() == 21)
    {
        // BUG FIX: original read 16-bit values as (low | high << 8) using
        //   Wire.read() | (Wire.read() << 8)
        // The C standard does NOT guarantee left-to-right evaluation of
        // sub-expressions in a bitwise-OR.  Both reads can execute in any
        // order, silently swapping bytes on some compilers.
        // Fix: read into named temporaries before combining.
        uint8_t lo, hi;

        lo = Wire.read(); hi = Wire.read();
        NVM_PAR_T1 = (uint16_t)(lo | (hi << 8));

        lo = Wire.read(); hi = Wire.read();
        NVM_PAR_T2 = (uint16_t)(lo | (hi << 8));

        NVM_PAR_T3  = (int8_t)Wire.read();

        lo = Wire.read(); hi = Wire.read();
        NVM_PAR_P1 = (int16_t)(lo | (hi << 8));

        lo = Wire.read(); hi = Wire.read();
        NVM_PAR_P2 = (int16_t)(lo | (hi << 8));

        NVM_PAR_P3  = (int8_t)Wire.read();
        NVM_PAR_P4  = (int8_t)Wire.read();

        lo = Wire.read(); hi = Wire.read();
        NVM_PAR_P5 = (uint16_t)(lo | (hi << 8));

        lo = Wire.read(); hi = Wire.read();
        NVM_PAR_P6 = (uint16_t)(lo | (hi << 8));

        NVM_PAR_P7  = (int8_t)Wire.read();
        NVM_PAR_P8  = (int8_t)Wire.read();

        lo = Wire.read(); hi = Wire.read();
        NVM_PAR_P9 = (int16_t)(lo | (hi << 8));

        NVM_PAR_P10 = (int8_t)Wire.read();
        NVM_PAR_P11 = (int8_t)Wire.read();

        delay(50);

        PAR_T1 = (float)NVM_PAR_T1 * 256.0f;
        PAR_T2 = (float)NVM_PAR_T2 / 1073741824.0f;
        PAR_T3 = (float)NVM_PAR_T3 / 281474976710656.0f;

        PAR_P1  = ((float)NVM_PAR_P1 - 16384.0f) / 1048576.0f;
        PAR_P2  = ((float)NVM_PAR_P2 - 16384.0f) / 536870912.0f;
        PAR_P3  = (float)NVM_PAR_P3  / 4294967296.0f;
        PAR_P4  = (float)NVM_PAR_P4  / 137438953472.0f;
        PAR_P5  = (float)NVM_PAR_P5  * 8.0f;
        PAR_P6  = (float)NVM_PAR_P6  / 64.0f;
        PAR_P7  = (float)NVM_PAR_P7  / 256.0f;
        PAR_P8  = (float)NVM_PAR_P8  / 32768.0f;
        PAR_P9  = (float)NVM_PAR_P9  / 281474976710656.0f;
        PAR_P10 = (float)NVM_PAR_P10 / 281474976710656.0f;
        PAR_P11 = (float)NVM_PAR_P11 / 36893488147419103232.0f; // 2^65
    }
}

void readTempPres(uint32_t *p, uint32_t *t)
{
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x04);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP388ADDRESS, 6);

    if (Wire.available() == 6)
    {
        uint8_t b0, b1, b2;

        b0 = Wire.read(); b1 = Wire.read(); b2 = Wire.read();
        *p = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16);

        b0 = Wire.read(); b1 = Wire.read(); b2 = Wire.read();
        *t = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16);
    }
}

float BMP388CompensateTemp(uint32_t uncompTemp)
{
    float pd1 = (float)uncompTemp - PAR_T1;
    float pd2 = pd1 * PAR_T2;
    return pd2 + (pd1 * pd1) * PAR_T3;
}

float BMP388CompensatePressure(uint32_t uncompPressure, float T)
{
    float pd1, pd2, pd3, pd4;

    pd1 = PAR_P6 * T;
    pd2 = PAR_P7 * (T * T);
    pd3 = PAR_P8 * (T * T * T);
    float pout1 = PAR_P5 + pd1 + pd2 + pd3;

    pd1 = PAR_P2 * T;
    pd2 = PAR_P3 * (T * T);
    pd3 = PAR_P4 * (T * T * T);
    float pout2 = (float)uncompPressure * (PAR_P1 + pd1 + pd2 + pd3);

    float up2  = (float)uncompPressure * (float)uncompPressure;
    float up3  = up2 * (float)uncompPressure;
    pd1 = up2;
    pd2 = PAR_P9 + (PAR_P10 * T);
    pd3 = pd1 * pd2;
    pd4 = pd3 + up3 * PAR_P11;

    return pout1 + pout2 + pd4;
}

void getInitialPressure(int sampleSize)
{
    // Reset software filter state so arming does not blend stale pre-arm values
    filterSeeded    = false;
    ema1            = 0.0f;
    ema2            = 0.0f;
    altitudeOut     = 0.0f;
    prev1           = 0.0f;
    prev2           = 0.0f;

    initialPressure = 0.0f;
    initialTemp     = 0.0f;
    int counter     = 0;

    for (int i = 0; i < sampleSize; i++)
    {
        uint32_t tStart = millis();
        uint8_t  status = 0;
        do {
            Wire.beginTransmission(BMP388ADDRESS);
            Wire.write(0x03);
            Wire.endTransmission(false);
            Wire.requestFrom(BMP388ADDRESS, 1);
            status = Wire.read();
            if (!(status & 0x20)) delay(5);
        } while (!(status & 0x20) && (millis() - tStart) < 150);

        if (!(status & 0x20)) continue;  // timed out — skip sample

        readTempPres(&rawPressure, &rawTemp);
        float t          = BMP388CompensateTemp(rawTemp);
        initialTemp     += t;
        initialPressure += BMP388CompensatePressure(rawPressure, t);
        counter++;
    }

    if (counter > 0)
    {
        initialPressure /= (float)counter;
        initialTemp     /= (float)counter;
    }
}

void updateAltitudeReadings()
{
    // Gate on pressure-ready flag
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP388ADDRESS, 1);
    uint8_t status = Wire.read();
    if (!(status & 0x20)) return;

    readTempPres(&rawPressure, &rawTemp);
    temp     = BMP388CompensateTemp(rawTemp);
    pressure = BMP388CompensatePressure(rawPressure, temp);

    if (initialPressure <= 0.0f) return;

    // Step 1: hypsometric altitude
    float rawAlt = 44330.0f * (1.0f - expf(0.1903f * logf(pressure / initialPressure)));

    // Step 2: median-of-3 spike rejection
    float medAlt = median3(rawAlt, prev1, prev2);
    prev2 = prev1;
    prev1 = rawAlt;

    // Step 3: cascaded EMA
    if (!filterSeeded)
    {
        ema1         = medAlt;
        ema2         = medAlt;
        altitudeOut  = medAlt;
        filterSeeded = true;
    }
    ema1 = EMA1_ALPHA * ema1 + (1.0f - EMA1_ALPHA) * medAlt;
    ema2 = EMA2_ALPHA * ema2 + (1.0f - EMA2_ALPHA) * ema1;

    // Step 4: thermal drift correction
    //   Altitude reads high when die temperature rises (gas law + board heat).
    //   correction = THERMAL_DRIFT_GAIN × (T_now − T_ref)
    float driftCorrection = THERMAL_DRIFT_GAIN * (temp - initialTemp);
    float correctedAlt    = ema2 - driftCorrection;

    // Step 5: hysteresis gate — suppress sub-noise-floor jitter
    if (fabsf(correctedAlt - altitudeOut) > ALTITUDE_HYSTERESIS_M)
    {
        altitudeOut = correctedAlt;
    }

    altitude = altitudeOut;
}
