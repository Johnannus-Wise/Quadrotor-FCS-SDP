#include "bmp388.h"

void initBMP388()
{
    Serial.println("1");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1B);  // PWR CONTROL REGISTER
    Wire.write(0x00);  // SET TO SLEEP MODE and ENABLE TEMP AND PRESSURE SENSORS
    Wire.endTransmission();
    delay(20);

    Serial.println("2");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1C);  // OVER SAMPLING RESOLUTION REGISTER (osr_p and t)
    Wire.write(0x03);  // SETTING THE TEMP RESOLUTION TO x1 AND PRESSURE TO x8
    Wire.endTransmission();
    delay(20);

    Serial.println("3");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1D);  // Oversampling register config
    Wire.write(0x02);  // setting the sampling period to 20ms
    Wire.endTransmission();
    delay(20);

    Serial.println("4");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1F);  // IIR FILTER REGISTER CONFIG
    Wire.write(0x04);  // SET IIR COEFFICENT TO 3
    Wire.endTransmission();
    delay(20);

    Serial.println("5");
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x1B);  // PWR CONTROL REGISTER
    Wire.write(0x33);  // SET TO NORMAL MODE and ENABLE TEMP AND PRESSURE SENSORS
    Wire.endTransmission();
    delay(20);

    Serial.println("I WILL GET COMPENSATION DATA");
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
    int8_t   NVM_PAR_T3, NVM_PAR_P3, NVM_PAR_P4, NVM_PAR_P7, NVM_PAR_P8, NVM_PAR_P10, NVM_PAR_P11;
    int16_t  NVM_PAR_P1, NVM_PAR_P2, NVM_PAR_P9;
    uint16_t NVM_PAR_T1, NVM_PAR_T2, NVM_PAR_P5, NVM_PAR_P6;

    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x31);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP388ADDRESS, 21);
    delay(500);

    Serial.println("I AM ABOUT TO READ");
    if (Wire.available() == 21)
    {
        Serial.println("I AM READING");
        NVM_PAR_T1 = Wire.read() | (Wire.read() << 8);
        NVM_PAR_T2 = Wire.read() | (Wire.read() << 8);
        NVM_PAR_T3 = (int8_t)Wire.read();
        NVM_PAR_P1 = Wire.read() | (Wire.read() << 8);
        NVM_PAR_P2 = Wire.read() | (Wire.read() << 8);
        NVM_PAR_P3 = (int8_t)Wire.read();
        NVM_PAR_P4 = (int8_t)Wire.read();
        NVM_PAR_P5 = Wire.read() | (Wire.read() << 8);
        NVM_PAR_P6 = Wire.read() | (Wire.read() << 8);
        NVM_PAR_P7 = (int8_t)Wire.read();
        NVM_PAR_P8 = (int8_t)Wire.read();
        NVM_PAR_P9 = Wire.read() | (Wire.read() << 8);
        NVM_PAR_P10 = (int8_t)Wire.read();
        NVM_PAR_P11 = (int8_t)Wire.read();
        delay(50);

        PAR_T1 = (float)NVM_PAR_T1 * 256.0f;
        PAR_T2 = (float)NVM_PAR_T2 / 1073741824.0f;
        PAR_T3 = (float)NVM_PAR_T3 / 281474976710656.0f;

        PAR_P1 = ((float)NVM_PAR_P1 - 16384.0f) / 1048576.0f;
        PAR_P2 = ((float)NVM_PAR_P2 - 16384.0f) / 536870912.0f;
        PAR_P3 = (float)NVM_PAR_P3 / 4294967296.0f;

        PAR_P4  = (float)NVM_PAR_P4 / 137438953472.0f;
        PAR_P5  = (float)NVM_PAR_P5 * 8.0f;
        PAR_P6  = (float)NVM_PAR_P6 / 64.0f;

        PAR_P7  = (float)NVM_PAR_P7 / 256.0f;
        PAR_P8  = (float)NVM_PAR_P8 / 32768.0f;
        PAR_P9  = (float)NVM_PAR_P9 / 281474976710656.0f;

        PAR_P10 = (float)NVM_PAR_P10 / 281474976710656.0f;
        PAR_P11 = (float)NVM_PAR_P11 / pow(2.0, 65);
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
        *p = (Wire.read() | (Wire.read() << 8) | (Wire.read() << 16));
        *t = (Wire.read() | (Wire.read() << 8) | (Wire.read() << 16));
    }
}

float BMP388CompensatTemp(uint32_t uncompTemp)
{
    float partial_dataT1 = (float)(uncompTemp - PAR_T1);
    float partial_dataT2 = (float)(partial_dataT1 * PAR_T2);
    float t_lin          = partial_dataT2 + (partial_dataT1 * partial_dataT1) * PAR_T3;
    return t_lin;
}

float BMP388CompensatePressure(uint32_t uncompPressure, float calibratedTemp)
{
    float partial_data1, partial_data2, partial_data3, partial_data4;
    float partial_out1, partial_out2;

    partial_data1 = PAR_P6 * calibratedTemp;
    partial_data2 = PAR_P7 * (calibratedTemp * calibratedTemp);
    partial_data3 = PAR_P8 * (calibratedTemp * calibratedTemp * calibratedTemp);
    partial_out1  = PAR_P5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = PAR_P2 * calibratedTemp;
    partial_data2 = PAR_P3 * (calibratedTemp * calibratedTemp);
    partial_data3 = PAR_P4 * (calibratedTemp * calibratedTemp * calibratedTemp);
    partial_out2  = (float)uncompPressure * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncompPressure * (float)uncompPressure;
    partial_data2 = PAR_P9 + (PAR_P10 * calibratedTemp);
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3
                  + ((float)uncompPressure * (float)uncompPressure * (float)uncompPressure) * PAR_P11;

    return partial_out1 + partial_out2 + partial_data4;
}

void getInitialPressure(int sampleSize)
{
    initialPressure = 0;
    int counter = 0;
    for (int i = 0; i < sampleSize; i++)
    {
        readTempPres(&rawPressure, &rawTemp);
        initialTemp     = BMP388CompensatTemp(rawTemp);
        initialPressure = initialPressure + BMP388CompensatePressure(rawPressure, initialTemp);
        counter++;
    }
    initialPressure = initialPressure / counter;
}

void updateAltitudeReadings()
{
    Wire.beginTransmission(BMP388ADDRESS);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP388ADDRESS, 1);
    uint8_t status = Wire.read();

    if (!(status & 0x20)) return;  // Pressure not ready, skip this cycle

    readTempPres(&rawPressure, &rawTemp);
    temp     = BMP388CompensatTemp(rawTemp);
    pressure = BMP388CompensatePressure(rawPressure, temp);
    altitude = 44330.0f * (1.0f - expf(0.1903f * logf(pressure / initialPressure)));
}
