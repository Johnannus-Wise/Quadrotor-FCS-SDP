#define _CRT_SECURE_NO_WARNINGS

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PORT "\\\\.\\COM4"
#define BAUDRATE 460800
#define CSV_FILE "flight_log.csv"

typedef struct
{
    char timestamp[64];

    double Pitch, Roll;
    double AccelX, AccelY, AccelZ;
    double GyroX, GyroY, GyroZ;

    double Altitude, Pressure, Temperature;

    int ThrottleInput;
    int DesiredPitch;
    int DesiredRoll;
    int YawSpeed;
    int DesiredAltitude;

    double RollAngle_Output;
    double PitchAngle_Output;
    double RollRate_Output;
    double PitchRate_Output;
    double YawRate_Output;
    double Altitude_Output;

    char AltitudeLock[16];
    char AngleMode[16];

    int Motor_FL;
    int Motor_FR;
    int Motor_RR;
    int Motor_RL;

    int ReferencePitch;
    int ReferenceRoll;

} FlightSample;

static HANDLE openSerial(void)
{
    HANDLE hSerial = CreateFileA(
        PORT,
        GENERIC_READ,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL);

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        printf("Failed to open %s\n", PORT);
        return INVALID_HANDLE_VALUE;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);

    GetCommState(hSerial, &dcb);

    dcb.BaudRate = BAUDRATE;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;

    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS t = {0};
    t.ReadIntervalTimeout = 1;
    t.ReadTotalTimeoutConstant = 1;
    t.ReadTotalTimeoutMultiplier = 0;

    SetCommTimeouts(hSerial, &t);

    SetupComm(hSerial, 262144, 262144);

    PurgeComm(hSerial,
        PURGE_RXCLEAR |
        PURGE_TXCLEAR);

    return hSerial;
}

static void writeHeader(FILE* f)
{
    fprintf(f,
        "Pitch,Roll,"
        "AccelX,AccelY,AccelZ,"
        "GyroX,GyroY,GyroZ,"
        "Altitude,Pressure,Temperature,"
        "ThrottleInput,DesiredPitch,DesiredRoll,YawSpeed,DesiredAltitude,"
        "RollAngle_Output,PitchAngle_Output,"
        "RollRate_Output,PitchRate_Output,"
        "YawRate_Output,Altitude_Output,"
        "AltitudeLock,AngleMode,"
        "Motor_FL,Motor_FR,Motor_RR,Motor_RL,"
        "ReferencePitch,ReferenceRoll\n");
}

static void writeSample(FILE* f, FlightSample* s)
{
    fprintf(f,
        "%.2f,%.2f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%d,%d,%d,%d,%d,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%s,%s,"
        "%d,%d,%d,%d,"
        "%d,%d\n",

        s->Pitch,
        s->Roll,

        s->AccelX,
        s->AccelY,
        s->AccelZ,

        s->GyroX,
        s->GyroY,
        s->GyroZ,

        s->Altitude,
        s->Pressure,
        s->Temperature,

        s->ThrottleInput,
        s->DesiredPitch,
        s->DesiredRoll,
        s->YawSpeed,
        s->DesiredAltitude,

        s->RollAngle_Output,
        s->PitchAngle_Output,
        s->RollRate_Output,
        s->PitchRate_Output,
        s->YawRate_Output,
        s->Altitude_Output,

        s->AltitudeLock,
        s->AngleMode,

        s->Motor_FL,
        s->Motor_FR,
        s->Motor_RR,
        s->Motor_RL,

        s->ReferencePitch,
        s->ReferenceRoll
    );
}

static void parseLine(char* line,
                      FlightSample* sample,
                      FILE* csv)
{
    if (strstr(line, "Pitch:") &&
        strstr(line, "Accel:") &&
        strstr(line, "Gyro:"))
    {
        sscanf(line,
            "Pitch: %lf° , Roll: %lf° , Accel: (%lf, %lf, %lf) , Gyro: (%lf, %lf, %lf)",
            &sample->Pitch,
            &sample->Roll,
            &sample->AccelX,
            &sample->AccelY,
            &sample->AccelZ,
            &sample->GyroX,
            &sample->GyroY,
            &sample->GyroZ);

        return;
    }

    if (strstr(line, "Altitude:"))
    {
        sscanf(line,
            "Altitude: %lf m, Pressure: %lf Pa, Temperature: %lf",
            &sample->Altitude,
            &sample->Pressure,
            &sample->Temperature);

        return;
    }

    if (strstr(line, "mainThrottleInput"))
    {
        sscanf(line,
            "mainThrottleInput: %d, Desired Pitch: %d, Desired Roll: %d, Yaw Speed: %d, Desired Altitude: %d",
            &sample->ThrottleInput,
            &sample->DesiredPitch,
            &sample->DesiredRoll,
            &sample->YawSpeed,
            &sample->DesiredAltitude);

        return;
    }

    if (strstr(line, "PID Outputs"))
    {
        sscanf(line,
            "PID Outputs Roll Angle: %lf, Pitch Angle: %lf, Roll Rate: %lf, Pitch Rate: %lf, Yaw Rate: %lf, Altitude: %lf",
            &sample->RollAngle_Output,
            &sample->PitchAngle_Output,
            &sample->RollRate_Output,
            &sample->PitchRate_Output,
            &sample->YawRate_Output,
            &sample->Altitude_Output);

        return;
    }

    if (strstr(line, "Altitude Lock"))
    {
        sscanf(line,
            "Altitude Lock: %15[^,], Angle Mode: %15s",
            sample->AltitudeLock,
            sample->AngleMode);

        return;
    }

    if (strstr(line, "motor outputs"))
    {
        sscanf(line,
            "motor outputs: FL %d, FR %d, RR %d, RL %d, Reference Pitch: %d, Reference Roll: %d",
            &sample->Motor_FL,
            &sample->Motor_FR,
            &sample->Motor_RR,
            &sample->Motor_RL,
            &sample->ReferencePitch,
            &sample->ReferenceRoll);

        writeSample(csv, sample);

        memset(sample, 0, sizeof(FlightSample));
    }
}

int main(void)
{
    HANDLE serial = openSerial();

    if (serial == INVALID_HANDLE_VALUE)
        return -1;

    FILE* csv = fopen(CSV_FILE, "w");

    if (!csv)
        return -1;

    setvbuf(csv, NULL, _IOFBF, 1024 * 1024);

    writeHeader(csv);

    printf("Connected to COM4 @ %d\n", BAUDRATE);

    FlightSample sample = {0};

    char line[4096];
    int index = 0;

    BYTE rx[8192];
    DWORD bytesRead;

    while (1)
    {
        if (!ReadFile(
                serial,
                rx,
                sizeof(rx),
                &bytesRead,
                NULL))
            continue;

        for (DWORD i = 0; i < bytesRead; i++)
        {
            char c = (char)rx[i];

            if (c == '\n')
            {
                line[index] = 0;

                parseLine(line,
                          &sample,
                          csv);

                index = 0;
            }
            else if (index < sizeof(line) - 1)
            {
                line[index++] = c;
            }
        }
    }

    fclose(csv);
    CloseHandle(serial);

    return 0;
}