#include "icm45686.h"

void ICM45686_Init()
{
    Wire.beginTransmission(ICMAddress);
    Wire.write(0x72);  // WHO_AM_I register
    Wire.endTransmission(false);
    Wire.requestFrom(ICMAddress, 1);

    if (Wire.available() == 1)
    {
        Serial.printf("The WHO AM I reads: %X\n", Wire.read());
    }
    delay(10);

    I2CWriteIREG(0xA6, 0xA400, 0x20);  // Enabling gyro AAF, Disabling interpolator
    delay(10);
    Serial.printf("The data written at 0xA6 is: 0x%X \n", I2CReadIREG(0xA6, 0xA400));
    delay(10);

    I2CWriteIREG(0x7B, 0xA400, 0x01);  // Enabling accel AAF, Disabling interpolator
    delay(10);
    Serial.printf("The data written at 0x7B is: 0x%X \n", I2CReadIREG(0x7B, 0xA400));
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x1B);  // select register
    Wire.write(0x34);  // setting accel to +-4g and ODR to 3.2KHz
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x1C);  // select register
    Wire.write(0x34);  // setting gyro to +-500 dps and ODR to 3.2KHz
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x10);  // select register
    Wire.write(0x0F);  // setting both gyro and accel to low noise mode
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x1D);  // select register
    Wire.write(0x5F);  // FIFO config0: stream mode, 8KB depth
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x22);  // select register
    Wire.write(0x00);  // FIFO config4: disable compression, disable timestamp
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x21);  // select register
    Wire.write(0x07);  // FIFO config3: disable high resolution
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x2F);  // select register
    Wire.write(0x00);  // IOC PAD SCENARIO: disable AUX1
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x3A);  // select register
    Wire.write(0x00);  // APEX CONFIG1: disable all interrupt generations
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x10);  // select register
    Wire.write(0x0F);  // INT2 CONFIG0: disable all interrupts
    Wire.endTransmission();
    delay(10);
}

void updateIMUReadings()
{
    I2CReadBurst(0x00, ICMAddress, 12);

    accelX = (rawAccelX * 9.8) / accelSensitivity;
    accelY = (rawAccelY * 9.8) / accelSensitivity;
    accelZ = (rawAccelZ * 9.8) / accelSensitivity;

    gyroX = (float)(rawGyroX / gyroSensitivity);
    gyroY = (float)(rawGyroY / gyroSensitivity);
    gyroZ = (float)(-rawGyroZ / gyroSensitivity);

    accelPitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
    accelRoll  = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

    pitch = pitch + (gyroX * deltaTime);
    roll  = roll  + (gyroY * deltaTime);

    pitch = alphaIMU * pitch + (1.0 - alphaIMU) * accelPitch;
    roll  = alphaIMU * roll  + (1.0 - alphaIMU) * accelRoll;
}

// ============================================================
//  I2C helpers
// ============================================================

void I2CWriteIREG(uint16_t target_register, uint16_t base_increment, uint8_t data)
{
    target_register     = target_register + base_increment;
    uint8_t upperAddress = target_register >> 8;
    uint8_t lowerAddress = target_register;

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x7C);       // IREG register high address
    Wire.write(upperAddress);
    Wire.write(lowerAddress);
    Wire.write(data);
    uint8_t err = Wire.endTransmission();
    if (err != 0)
    {
        Serial.printf("I2C Write Error: %d\n", err);
    }
}

uint8_t I2CReadIREG(uint16_t target_register, uint16_t base_increment)
{
    target_register     = target_register + base_increment;
    uint8_t upperAddress = target_register >> 8;
    uint8_t lowerAddress = target_register;

    Wire.beginTransmission(ICMAddress);
    Wire.write(0x7C);
    Wire.write(upperAddress);
    Wire.write(lowerAddress);
    Wire.endTransmission(false);

    delayMicroseconds(5);
    Wire.requestFrom(ICMAddress, 1);
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

    // Little Endian
    rawAccelX  = Wire.read();
    rawAccelX  = (Wire.read() << 8) | rawAccelX;
    rawAccelY  = Wire.read();
    rawAccelY  = (Wire.read() << 8) | rawAccelY;
    rawAccelZ  = Wire.read();
    rawAccelZ  = (Wire.read() << 8) | rawAccelZ;

    rawGyroX   = Wire.read();
    rawGyroX   = (Wire.read() << 8) | rawGyroX;
    rawGyroY   = Wire.read();
    rawGyroY   = (Wire.read() << 8) | rawGyroY;
    rawGyroZ   = Wire.read();
    rawGyroZ   = (Wire.read() << 8) | rawGyroZ;
}
