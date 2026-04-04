#include "icm45686.h"

void ICM45686_Init()
{
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x72);   // WHO_AM_I
    Wire.endTransmission(false);
    Wire.requestFrom(ICM_ADDRESS, 1);
    if (Wire.available() == 1)
    {
        uint8_t whoAmI = Wire.read();
        Serial.printf("ICM-45686 WHO_AM_I = 0x%02X (expect 0xE9)\n", whoAmI);
    }
    delay(10);

    // Gyro AAF enable, interpolator disabled
    I2CWriteIREG(0xA6, 0xA400, 0x20);
    delay(10);

    // Accel AAF enable, interpolator disabled
    I2CWriteIREG(0x7B, 0xA400, 0x01);
    delay(10);

    // Accel config: ±4 g, 3.2 kHz ODR
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x1B);
    Wire.write(0x34);
    Wire.endTransmission();
    delay(10);

    // Gyro config: ±500 dps, 3.2 kHz ODR
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x1C);
    Wire.write(0x34);
    Wire.endTransmission();
    delay(10);

    // Low-noise mode for both gyro and accel
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x10);
    Wire.write(0x0F);
    Wire.endTransmission();
    delay(10);

    // FIFO config0: stream mode, 8 KB depth
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x1D);
    Wire.write(0x5F);
    Wire.endTransmission();
    delay(10);

    // FIFO config4: disable compression and timestamp
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x22);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // FIFO config3: disable high-resolution mode
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x21);
    Wire.write(0x07);
    Wire.endTransmission();
    delay(10);

    // IOC pad scenario: disable AUX1
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x2F);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // APEX config: disable all interrupt generation
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x3A);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);
}

void updateIMUReadings()
{
    I2CReadBurst(0x00, ICM_ADDRESS, 12);

    // Convert raw counts to SI units
    // BUG FIX: ACCEL_SENSITIVITY and GYRO_SENSITIVITY are now float literals
    //          (see config.h). Division of int16 by int caused truncation.
    accelX = (float)rawAccelX * 9.80665f / ACCEL_SENSITIVITY;
    accelY = (float)rawAccelY * 9.80665f / ACCEL_SENSITIVITY;
    accelZ = (float)rawAccelZ * 9.80665f / ACCEL_SENSITIVITY;

    // BUG FIX: explicit float cast before division — rawGyroX is int16_t and
    //          GYRO_SENSITIVITY is now float, but the cast makes intent clear
    //          and avoids surprising promotion rules on strict compilers.
    gyroX =  (float)rawGyroX / GYRO_SENSITIVITY;
    gyroY =  (float)rawGyroY / GYRO_SENSITIVITY;
    gyroZ = -(float)rawGyroZ / GYRO_SENSITIVITY;  // negated to match frame convention

    // Accelerometer-derived angles (degrees)
    accelPitch = atan2f(accelY, sqrtf(accelX * accelX + accelZ * accelZ)) * 180.0f / (float)PI;
    accelRoll  = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ)) * 180.0f / (float)PI;

    // Complementary filter — integrates gyro then blends with accel
    pitch = pitch + (gyroX * deltaTime);
    roll  = roll  + (gyroY * deltaTime);

    pitch = alphaIMU * pitch + (1.0f - alphaIMU) * accelPitch;
    roll  = alphaIMU * roll  + (1.0f - alphaIMU) * accelRoll;
}

// ============================================================
//  I2C helpers
// ============================================================

void I2CWriteIREG(uint16_t target_register, uint16_t base_increment, uint8_t data)
{
    uint16_t addr        = target_register + base_increment;
    uint8_t  upperAddress = (uint8_t)(addr >> 8);
    uint8_t  lowerAddress = (uint8_t)(addr);

    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x7C);
    Wire.write(upperAddress);
    Wire.write(lowerAddress);
    Wire.write(data);
    uint8_t err = Wire.endTransmission();
    if (err != 0)
        Serial.printf("I2C Write IREG Error: %d\n", err);
}

uint8_t I2CReadIREG(uint16_t target_register, uint16_t base_increment)
{
    uint16_t addr        = target_register + base_increment;
    uint8_t  upperAddress = (uint8_t)(addr >> 8);
    uint8_t  lowerAddress = (uint8_t)(addr);

    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(0x7C);
    Wire.write(upperAddress);
    Wire.write(lowerAddress);
    Wire.endTransmission(false);

    delayMicroseconds(5);
    Wire.requestFrom(ICM_ADDRESS, 1);
    uint8_t returnData = Wire.read();
    Wire.endTransmission();
    return returnData;
}

void I2CReadBurst(uint8_t startReg, uint8_t deviceAddress, uint8_t burst)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(startReg);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddress, burst);

    // BUG FIX: original used expressions like:
    //   rawAccelX = Wire.read();
    //   rawAccelX = (Wire.read() << 8) | rawAccelX;
    // The second line reads a new byte and ORs it with the value already in
    // rawAccelX — this is correct logically, but relies on rawAccelX being
    // read before the left-hand side is written, which is not guaranteed
    // by the C standard (unsequenced side effect on the same object within
    // a full expression).  Some compilers evaluate the RHS Wire.read() calls
    // out of order.  Fix: use named temporaries.
    uint8_t lo, hi;

    lo = Wire.read(); hi = Wire.read();
    rawAccelX = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));

    lo = Wire.read(); hi = Wire.read();
    rawAccelY = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));

    lo = Wire.read(); hi = Wire.read();
    rawAccelZ = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));

    lo = Wire.read(); hi = Wire.read();
    rawGyroX  = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));

    lo = Wire.read(); hi = Wire.read();
    rawGyroY  = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));

    lo = Wire.read(); hi = Wire.read();
    rawGyroZ  = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}
