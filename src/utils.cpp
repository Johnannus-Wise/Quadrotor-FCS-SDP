#include "utils.h"
#include "globals.h"

float calSlope(float y2, float y1, float x2, float x1)
{
    // BUG FIX: original had no guard against x2 == x1 (division by zero).
    if (fabsf(x2 - x1) < 1e-9f) return 0.0f;
    return (y2 - y1) / (x2 - x1);
}

float calYIntercept(float slope, float x, float y)
{
    return y - (slope * x);
}

void flightLog() {
    //Sensor Readings
    Serial.printf("Pitch: %.2f°, Roll: %.2f°, ", pitch, roll);
    // Serial.printf("Pitch: %.2f°, Roll: %.2f°, Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)\n",
    //               pitch, roll, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    // Serial.printf("Altitude: %.2f m, Pressure: %.2f Pa, Temperature: %.2f °C\n",
    //               altitude, pressure, temp);
    //Pilot commands
    // Serial.printf("mainThrottleInput: %i, Desired Pitch: %i°, Desired Roll: %i°, Yaw Speed: %i°/s, Desired Altitude: %i m\n",
    //               mainThrottleInput, desiredPitch, desiredRoll, yawSpeed, desiredAltitude);
    //PID Gains
    // Serial.printf("Roll Angle PID: [%.2f, %.2f, %.2f], Pitch Angle PID: [%.2f, %.2f, %.2f], Roll Rate PID: [%.2f, %.2f, %.2f], Pitch Rate PID: [%.2f, %.2f, %.2f], Yaw Rate PID: [%.2f, %.2f, %.2f], Altitude PID: [%.2f, %.2f, %.2f]\n",
    //               rollAngleController.Kp, rollAngleController.Ki, rollAngleController.Kd,
    //               pitchAngleController.Kp, pitchAngleController.Ki, pitchAngleController.Kd,
    //               rollRateController.Kp, rollRateController.Ki, rollRateController.Kd,
    //               pitchRateController.Kp, pitchRateController.Ki, pitchRateController.Kd,
    //               yawRateController.Kp, yawRateController.Ki, yawRateController.Kd,
    //               altitudeController.Kp, altitudeController.Ki, altitudeController.Kd);
    //PID Outputs
    // Serial.printf("PID Outputs — Roll Angle: %.2f, Pitch Angle: %.2f, Roll Rate: %.2f, Pitch Rate: %.2f, Yaw Rate: %.2f, Altitude: %.2f\n",
    //               rollAngleController.PID_output, pitchAngleController.PID_output,
    //               rollRateController.PID_output, pitchRateController.PID_output,
    //               yawRateController.PID_output, altitudeController.PID_output);
    //Flight Mode
    // Serial.printf("Altitude Lock: %d, Angle Mode: %d\n", altitudeLock, manualControlled);
    //Motors
    Serial.printf("motor outputs: FL %d, FR %d, RR %d, RL %d, Reference Pitch: %i°, Reference Roll: %i°\n", motorMatrix[0], motorMatrix[1], motorMatrix[2], motorMatrix[3], desiredPitch, desiredRoll);
    // Serial.printf("mainThrottleInput: %d, Pitch: %.2f°, Roll: %.2f°, Accel: (%.2f, %.2f, %.2f), Gyro: (%.2f, %.2f, %.2f)\n", mainThrottleInput, pitch, roll, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    // channels.displayReadings();
}

void flightLog(bool plot) {
    if (plot) {
        Serial.print(">");

        Serial.print("Throttle Percent:");
        Serial.print(throttlePercentage);
        Serial.print(",");

        Serial.print("Pitch:");
        Serial.print(pitch);
        Serial.print(",");

        Serial.print("Roll:");
        Serial.print(roll);
        Serial.print(",");

        Serial.print("Altitude:");
        Serial.print(altitude);
        Serial.println();
    } else {
        flightLog();
    }
}