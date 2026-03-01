#pragma once
#include "config.h"
#include <Wire.h>

void ICM45686_Init();
void updateIMUReadings();

// I2C helpers
void    I2CWriteIREG(uint16_t target_register, uint16_t base_increment, uint8_t data);
uint8_t I2CReadIREG(uint16_t target_register, uint16_t base_increment);
void    I2CReadBurst(uint8_t startReg, uint8_t deviceAddress, uint8_t burst);
