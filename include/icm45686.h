#pragma once
#include "globals.h"
#include <Wire.h>

// ============================================================
//  Mahony filter state — defined in icm45686.cpp
//  Exposed here in case other modules need to read the quaternion
//  directly (e.g. for a 3D visualiser over serial).
// ============================================================
extern float q0, q1, q2, q3;
extern float gyroBiasX, gyroBiasY, gyroBiasZ;

void ICM45686_Init();
void updateIMUReadings();

// I2C indirect-register helpers (ICM-45686 IREG interface)
void    I2CWriteIREG(uint16_t target_register, uint16_t base_increment, uint8_t data);
uint8_t I2CReadIREG(uint16_t target_register, uint16_t base_increment);
void    I2CReadBurst(uint8_t startReg, uint8_t deviceAddress, uint8_t burst);
