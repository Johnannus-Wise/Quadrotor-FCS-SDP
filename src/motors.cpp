#include "motors.h"
#include "receiver.h"
bool manualControlled;
int hoverThrottle = maxThrottleValue * (1.8/4);

void movement()
{
    motorMixingAlgorithm(updatePID());
    clampMixedMotors();
    ledcWrite(frontLeft,  motorMatrix[0]);
    ledcWrite(frontRight, motorMatrix[1]);
    ledcWrite(rearRight,  motorMatrix[2]);
    ledcWrite(rearLeft,   motorMatrix[3]);
}

//returns true if the mixing is using altitude controller, false if manual throttle
bool updatePID()
{
    if (channels.ch[5].data < 500) {

        // ---- Angle Only (Manual Throttle) ----
        // Serial.println("Angle");
        if (altitudeLock)
        {
            altitudeLock = false;
        }
        rollAnglePID  = rollAngleController.compute(desiredRoll,  roll,  deltaTime, mainThrottleInput);
        pitchAnglePID = pitchAngleController.compute(desiredPitch, pitch, deltaTime, mainThrottleInput);
        rollRatePID   = rollRateController.compute(rollAnglePID,  gyroY, deltaTime, mainThrottleInput);
        pitchRatePID  = pitchRateController.compute(pitchAnglePID, gyroX, deltaTime, mainThrottleInput);
        yawRatePID    = yawRateController.compute(yawSpeed,       gyroZ, deltaTime, mainThrottleInput);
        manualControlled = true;
        return false;
    }
    else if (channels.ch[5].data > 1400)
    {
        // ---- Angle + Altitude (Autonomous Throttle) ----
        // Serial.println("Angle + Altitude");
        if (altitudeLock)
        {
            altitudeLock = false;
        }
        rollAnglePID  = rollAngleController.compute(desiredRoll,    roll,     deltaTime, mainThrottleInput);
        pitchAnglePID = pitchAngleController.compute(desiredPitch,  pitch,    deltaTime, mainThrottleInput);
        rollRatePID   = rollRateController.compute(rollAnglePID,    gyroY,    deltaTime, mainThrottleInput);
        pitchRatePID  = pitchRateController.compute(pitchAnglePID,  gyroX,    deltaTime, mainThrottleInput);
        yawRatePID    = yawRateController.compute(yawSpeed,         gyroZ,    deltaTime, mainThrottleInput);
        altitudePID   = altitudeController.compute(desiredAltitude, altitude, deltaTime, mainThrottleInput);
        manualControlled = false;
        return true;
    }
    else
    {
        // ---- Altitude Hold (Hover) ----
        // Serial.println("Hover");
        if (!altitudeLock)
        {
            hoverAltitude = altitude;
            altitudeLock  = true;
        }
        rollAnglePID  = rollAngleController.compute(desiredRoll,   roll,          deltaTime, mainThrottleInput);
        pitchAnglePID = pitchAngleController.compute(desiredPitch, pitch,         deltaTime, mainThrottleInput);
        rollRatePID   = rollRateController.compute(rollAnglePID,   gyroY,         deltaTime, mainThrottleInput);
        pitchRatePID  = pitchRateController.compute(pitchAnglePID, gyroX,         deltaTime, mainThrottleInput);
        yawRatePID    = yawRateController.compute(yawSpeed,        gyroZ,         deltaTime, mainThrottleInput);
        altitudePID   = altitudeController.compute(hoverAltitude,  altitude,      deltaTime, mainThrottleInput);
        manualControlled = false;
        return true;
    }
}

void motorMixingAlgorithm(bool altitudeControlled)
{
    if (!altitudeControlled) {
        //if greater than 80% throttle
        if (mainThrottleInput > maxThrottleValue * maxMainThrottleAllowed) {
            mainThrottleInput = maxThrottleValue * maxMainThrottleAllowed;
        }

        //if greater than 5% throttle use the PID controllers
        if (mainThrottleInput > maxThrottleValue * minThrottleRequirement) {
            motorMatrix[0] = mainThrottleInput + pitchRatePID + rollRatePID  - yawRatePID;
            motorMatrix[1] = mainThrottleInput + pitchRatePID - rollRatePID  + yawRatePID;
            motorMatrix[2] = mainThrottleInput - pitchRatePID - rollRatePID  - yawRatePID;
            motorMatrix[3] = mainThrottleInput - pitchRatePID + rollRatePID  + yawRatePID;
            for (int i = 0; i < 4; i++)
            {
                //if after mixing the motors would be slower than 5%, then clamp the motors to 5%
                if (motorMatrix[i] < (maxThrottleValue * minThrottleRequirement)) {
                    motorMatrix[i] = maxThrottleValue * minThrottleRequirement;
                }
                
            }
            
        }
        //if less than 5% then use direct throttle
        else {
            motorMatrix[0] = mainThrottleInput;
            motorMatrix[1] = mainThrottleInput;
            motorMatrix[2] = mainThrottleInput;
            motorMatrix[3] = mainThrottleInput;
        }



        // Floor motors at 55% of PWM range when throttle is above 10%
        for (int i = 0; i < 4; i++)
        {
            if (motorMatrix[i] < maxThrottleValue * minThrottleRequirement && mainThrottleInput > maxThrottleValue * minThrottleRequirement)
            {
                motorMatrix[i] = maxThrottleValue * minThrottleRequirement;
            }
        }
    }
    else
    {
        anti_Windup();
        if (altitudePID > maxThrottleValue * maxMainThrottleAllowed)
        {
            altitudePID = maxThrottleValue * maxMainThrottleAllowed;
        }
        if (altitudePID >= (hoverThrottle))
        motorMatrix[0] = altitudePID + pitchRatePID + rollRatePID  - yawRatePID;
        motorMatrix[1] = altitudePID + pitchRatePID - rollRatePID  + yawRatePID;
        motorMatrix[2] = altitudePID - pitchRatePID - rollRatePID  - yawRatePID;
        motorMatrix[3] = altitudePID - pitchRatePID + rollRatePID  + yawRatePID;
    }
}

void anti_Windup()
{
    if (altitudePID > maxMainThrottleAllowed * maxThrottleValue && altitudePID * altitudeController.error > 0)
    {
        altitudePID = altitudeController.unwind(deltaTime);
    }
}

void clampMixedMotors()
{
    for (int i = 0; i < 4; i++)
    {
        if (motorMatrix[i] > maxThrottleValue) motorMatrix[i] = maxThrottleValue;
        if (manualControlled && (motorMatrix[i] < maxThrottleValue * minThrottleRequirement) && mainThrottleInput >= maxThrottleValue * minThrottleRequirement) {
            motorMatrix[i] = maxThrottleValue * minThrottleRequirement;
            // Serial.printf("Condition1\n");
        } 
        else if (motorMatrix[i] < minThrottleValue) {
            motorMatrix[i] = minThrottleValue;
            // Serial.printf("Condition2\n");
        }
    }
}
