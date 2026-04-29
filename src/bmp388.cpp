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
#define BMP_IIR_REG_VALUE   0x06    // coeff 7

// Oversampling (reg 0x1C): bits[2:0] = pressure, bits[5:3] = temp (shifted)
//   Pressure: 0x00=x1 … 0x05=x32
//   x8 pressure: needs ODR ≤ 50 Hz.  Temp x1 keeps measurement time minimal.
#define BMP_OSR_PRESSURE    0x03    // x8 pressure
#define BMP_OSR_TEMP        0x00    // x1 temp (no shift needed — bits 5:3 = 0)
#define BMP_OSR_REG_VALUE   (BMP_OSR_TEMP | BMP_OSR_PRESSURE)

// ODR register (0x1D)
//   0x00=200Hz … 0x02=50Hz … 0x03=25Hz
//   x8 pressure resolution updates works with 50 Hz ODR
#define BMP_ODR_REG_VALUE   0x02    // 50Hz

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

// Complementary filter state for accelerometer-based altitude integration
static float accelAltitudeFiltered = 0.0f;  // low-pass filtered vertical accel residual
static float accelAltitudeAlpha = 0.05f;    // EMA coefficient for smoothing accel-derived alt

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

// International Standard Atmosphere (ISA) altitude formula
// Accurate up to ~11 km where troposphere ends
// Formula: h = T0/L_rate * (1 - (P/P0)^(R*L_rate/g*M))
// Simplified with ISA constants: h = 44330.77 * (1 - (P/P0)^(1/5.255))
//
// Parameters:
//   pressure_pa: measured pressure in Pascals
//   reference_pressure_pa: sea-level reference pressure in Pascals (typically 101325 Pa)
//   temperature_c: measured temperature in °C (for improved accuracy in non-standard conditions)
//
// Returns: altitude in meters relative to reference pressure
static inline float calculateAltitudeISA(float pressure_pa, float reference_pressure_pa, float temperature_c)
{
    // Guard against invalid inputs
    if (pressure_pa <= 0.0f || reference_pressure_pa <= 0.0f) return 0.0f;
    
    // ISA formula with temperature compensation
    // Temperature lapse rate in troposphere: 6.5 K per 1000 m (positive value)
    // Reference temperature at sea level: 288.15 K (15°C)
    const float ISA_TEMP_LAPSE_RATE = 0.0065f;        // K/m (positive)
    const float ISA_TEMP_SEA_LEVEL = 288.15f;          // K
    const float ISA_SCALE_HEIGHT = 44330.77f;          // meters (T0 / L_rate = 288.15 / 0.0065)
    const float ISA_EXPONENT = 1.0f / 5.255f;          // Exact: R*L/g*M ≈ 0.190263
    
    // Temperature in Kelvin
    float temp_kelvin = temperature_c + 273.15f;
    
    // Mean effective temperature for this measurement segment
    // Use average of ISA reference and measured temperature for improved accuracy
    float mean_temp = 0.5f * (ISA_TEMP_SEA_LEVEL + temp_kelvin);
    
    // Pressure ratio
    float pressure_ratio = pressure_pa / reference_pressure_pa;
    
    // ISA altitude formula: h = (T_mean / L_rate) * (1 - (P/P0)^exponent)
    // Temperature-adaptive: h = (mean_temp / ISA_TEMP_LAPSE_RATE) * (1 - (P/P0)^(1/5.255))
    float altitude = (mean_temp / ISA_TEMP_LAPSE_RATE) * (1.0f - powf(pressure_ratio, ISA_EXPONENT));
    
    return altitude;
}

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

void initBMP388()
{
    // Serial.println("1");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1B);  // PWR CONTROL REGISTER
    Wire.write(0x00);  // SET TO SLEEP MODE and ENABLE TEMP AND PRESSURE SENSORS
    Wire.endTransmission();
    delay(20);

    // Serial.println("2");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1C);  // OVER SAMPLING RESOLUTION REGISTER (osr_p and t)
    Wire.write(BMP_OSR_PRESSURE);  // SETTING THE TEMP RESOLUTION TO x1 AND PRESSURE TO x8
    Wire.endTransmission();
    delay(20);

    // Serial.println("3");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1D);  // Oversampling register config
    Wire.write(BMP_ODR_REG_VALUE);  // setting the sampling period to 20ms
    Wire.endTransmission();
    delay(20);

    // Serial.println("4");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1F);  // IIR FILTER REGISTER CONFIG
    Wire.write(BMP_IIR_REG_VALUE);  // SET IIR COEFFICENT TO 7
    Wire.endTransmission();
    delay(20);

    // Serial.println("5");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1B);  // PWR CONTROL REGISTER
    Wire.write(0x33);  // SET TO NORMAL MODE and ENABLE TEMP AND PRESSURE SENSORS
    Wire.endTransmission();
    delay(20);

    // Serial.println("I WILL GET COMPENSATION DATA");
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
    // Serial.println("Data Compensation:");
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
    // Serial.println("Compensation data obtained");
}

void readTempPres(uint32_t *p, uint32_t *t)
{
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x04);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP388ADDRESS, 6);

    if (Wire.available() == 6)
    {
        // Serial.println("Reading temp and pressure...");
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
    // Serial.println("Getting initial pressure...");
    // Reset software filter state so arming does not blend stale pre-arm values
    filterSeeded    = false;
    ema1            = 0.0f;
    ema2            = 0.0f;
    altitudeOut     = 0.0f;
    prev1           = 0.0f;
    prev2           = 0.0f;
    
    // Reset complementary filter state
    altitudeFromAccel = 0.0f;
    velocityZ = 0.0f;
    accelAltitudeFiltered = 0.0f;

    initialPressure = 0.0f;
    initialTemp     = 0.0f;
    int counter     = 0;

    for (int i = 0; i < sampleSize; i++)
    {
        uint32_t tStart = millis();
        uint8_t  status = 0;
        //Data ready check loop with 150 ms timeout (should be ~20 ms at 50 Hz ODR + settling time)
        do {
            Wire.beginTransmission(BMP388ADDRESS);
            Wire.write(0x03);
            Wire.endTransmission(false);
            Wire.requestFrom(BMP388ADDRESS, 1);
            status = Wire.read();
            if (!(status & 0x20)) {
                // Serial.println("Waiting for data...");
                delay(5);
            } 
        } while (!(status & 0x20) && (millis() - tStart) < 500);

        if (!(status & 0x20)) {
            // Serial.println("Data ready timeout — skipping sample");
            continue;  // timed out — skip sample
        }

        // Serial.println("Data ready — reading sample");
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
        // Serial.printf("Initial pressure = %.2f Pa | Initial temp = %.2f °C\n", initialPressure, initialTemp);
    }
    // Serial.println("Initial pressure obtained");
}

void renewReferencePressure(int sampleSize)
{
    // When motors are armed, establish a new baseline altitude
    // Reset all altitude filters and resample the current pressure reading
    // Serial.println("Renewing reference pressure at arm...");
    
    filterSeeded    = false;
    ema1            = 0.0f;
    ema2            = 0.0f;
    altitudeOut     = 0.0f;
    prev1           = 0.0f;
    prev2           = 0.0f;
    
    // Reset complementary filter state
    altitudeFromAccel = 0.0f;
    velocityZ = 0.0f;
    accelAltitudeFiltered = 0.0f;

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
            if (!(status & 0x20)) {
                delay(5);
            } 
        } while (!(status & 0x20) && (millis() - tStart) < 500);

        if (!(status & 0x20)) {
            continue;  // timed out — skip sample
        }

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
        // Serial.printf("Reference renewed: pressure = %.2f Pa | temp = %.2f °C\n", initialPressure, initialTemp);
    }
    // Reset altitude to zero when armed
    altitude = 0.0f;
    altitudeOut = 0.0f;
    // Serial.println("Reference pressure renewed at arm");
}

void updateAltitudeReadings()
{
    // Serial.println("Updating Altitude...");
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

    // ---- BAROMETER PATH ----
    // Step 1: ISA altitude calculation from pressure and temperature
    float rawAlt = calculateAltitudeISA(pressure, initialPressure, temp);

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
        altitudeFromAccel = medAlt;  // sync accel altitude to baro
    }
    ema1 = EMA1_ALPHA * ema1 + (1.0f - EMA1_ALPHA) * medAlt;
    ema2 = EMA2_ALPHA * ema2 + (1.0f - EMA2_ALPHA) * ema1;

    // Step 4: BMP388 chip thermal drift correction
    //   The ISA formula uses actual measured temperature, but BMP388 can internally
    //   heat up during long flights. This correction accounts for self-heating effects.
    //   Note: initialTemp is sampled at startup/arming and may differ from ambient
    //   if the chip has already warmed up. Consider this when tuning THERMAL_DRIFT_GAIN.
    float driftCorrection = THERMAL_DRIFT_GAIN * (temp - initialTemp);
    float baroAltitude    = ema2 - driftCorrection;

    // ---- ACCELEROMETER PATH (Complementary Filter Integration) ----
    // Estimate vertical acceleration: accelZ - gravity (net acceleration in inertial frame)
    // accelZ is in m/s² and already includes gravity when level
    float accelNetVertical = accelZ - GRAVITY;  // subtract gravity to get net vertical accel

    // Integrate acceleration to velocity
    velocityZ += accelNetVertical * deltaTime;

    // Integrate velocity to altitude
    altitudeFromAccel += velocityZ * deltaTime;

    // Low-pass filter the accelerometer altitude to reduce noise
    accelAltitudeFiltered = accelAltitudeAlpha * accelAltitudeFiltered + 
                           (1.0f - accelAltitudeAlpha) * altitudeFromAccel;

    // ---- COMPLEMENTARY FILTER ----
    // Blend barometer (low-pass, drift-free) with accelerometer (high-pass, responsive)
    // When altitudeComplementaryAlpha = 0: use barometer only
    // When altitudeComplementaryAlpha = 1: use accelerometer only
    // Typical range: 0.05 - 0.3 (mostly barometer with fast accel correction)
    float fusedAltitude = (1.0f - altitudeComplementaryAlpha) * baroAltitude + 
                         altitudeComplementaryAlpha * accelAltitudeFiltered;

    // Step 5: hysteresis gate — suppress sub-noise-floor jitter
    if (fabsf(fusedAltitude - altitudeOut) > ALTITUDE_HYSTERESIS_M)
    {
        altitudeOut = fusedAltitude;
    }

    altitude = altitudeOut;
    // Serial.println("Altitude updated");
}
