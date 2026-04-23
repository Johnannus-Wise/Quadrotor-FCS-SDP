#include "icm45686.h"

// Quaternion — represents current estimated orientation.
// Initialised as identity (no rotation).
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Accumulated gyro bias estimates (rad/s).
// The integral term of the PI controller builds these up over time
// and subtracts them from the raw gyro before integration.
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

// ============================================================

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

    // ---- Convert raw counts to SI units ----
    accelX =  (float)rawAccelX * GRAVITY / ACCEL_SENSITIVITY;
    accelY =  (float)rawAccelY * GRAVITY / ACCEL_SENSITIVITY;
    accelZ =  (float)rawAccelZ * GRAVITY / ACCEL_SENSITIVITY;

    // Gyro in deg/s — kept in deg/s for the PID rate controllers at the end.
    // A separate rad/s copy is used internally for the quaternion integration.
    gyroX =  (float)rawGyroX / GYRO_SENSITIVITY;
    gyroY =  (float)rawGyroY / GYRO_SENSITIVITY;
    gyroZ = -(float)rawGyroZ / GYRO_SENSITIVITY;

    // ---- Mahony filter ----

    // 1. Normalise the accelerometer reading.
    //    If norm is zero (sensor fault / free-fall) skip the correction step
    //    entirely — the quaternion will coast on gyro alone this cycle.
    float norm = sqrtf(accelX*accelX + accelY*accelY + accelZ*accelZ);
    if (norm == 0.0f) return;
    float ax = accelX / norm;
    float ay = accelY / norm;
    float az = accelZ / norm;

    // 2. Estimated gravity direction in body frame derived from the quaternion.
    //    This is the third column of the rotation matrix built from q.
    //    It answers: "given our current estimated orientation,
    //    which direction should gravity be pointing in sensor coordinates?"
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3. Cross product error: accel_measured × accel_expected.
    //    Direction = axis of rotation needed to align the two vectors.
    //    Magnitude ≈ sin(angle between them), so small errors stay small.
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    // 4. PI controller on the error.
    //    Integral accumulates the gyro bias estimate over time.
    gyroBiasX += ex * MAHONY_KI * deltaTime;
    gyroBiasY += ey * MAHONY_KI * deltaTime;
    gyroBiasZ += ez * MAHONY_KI * deltaTime;

    // Convert gyro to rad/s and apply correction before integrating.
    float gx = gyroX * (PI / 180.0f) + MAHONY_KP * ex + gyroBiasX;
    float gy = gyroY * (PI / 180.0f) + MAHONY_KP * ey + gyroBiasY;
    float gz = gyroZ * (PI / 180.0f) + MAHONY_KP * ez + gyroBiasZ;

    // 5. Integrate the quaternion derivative: q̇ = 0.5 * q ⊗ [0, gx, gy, gz]
    float dt = deltaTime;
    q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
    q1 += 0.5f * ( q0*gx + q2*gz - q3*gy) * dt;
    q2 += 0.5f * ( q0*gy - q1*gz + q3*gx) * dt;
    q3 += 0.5f * ( q0*gz + q1*gy - q2*gx) * dt;

    // 6. Renormalise the quaternion to prevent numerical drift.
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    // 7. Convert quaternion back to Euler angles (degrees) for PID controllers.
    roll = asinf ( 2.0f * (q0*q2 - q3*q1))                                  * (180.0f / PI);
    pitch  = atan2f( 2.0f * (q0*q1 + q2*q3),  1.0f - 2.0f*(q1*q1 + q2*q2))   * (180.0f / PI);
    yaw   = atan2f( 2.0f * (q0*q3 + q1*q2),  1.0f - 2.0f*(q2*q2 + q3*q3))   * (180.0f / PI);

    // gyroX/Y/Z remain in deg/s — used directly by the PID rate controllers.
}

// ============================================================
//  I2C helpers
// ============================================================

void I2CWriteIREG(uint16_t target_register, uint16_t base_increment, uint8_t data)
{
    uint16_t addr         = target_register + base_increment;
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
    uint16_t addr         = target_register + base_increment;
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