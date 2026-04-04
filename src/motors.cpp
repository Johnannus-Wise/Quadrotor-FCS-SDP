#include "motors.h"
#include "receiver.h"
#include "globals.h"

bool manualControlled;
int hoverThrottle = MAX_THROTTLE_VALUE * (1.8/4);

void movement()
{
    motorMixingAlgorithm(updatePID());
    clampMixedMotors();
    ledcWrite(FRONT_LEFT,  motorMatrix[0]);
    ledcWrite(FRONT_RIGHT, motorMatrix[1]);
    ledcWrite(REAR_RIGHT,  motorMatrix[2]);
    ledcWrite(REAR_LEFT,   motorMatrix[3]);
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
        rollAnglePID  = rollAngleController.compute(desiredRoll,  roll,  deltaTime);
        pitchAnglePID = pitchAngleController.compute(desiredPitch, pitch, deltaTime);
        rollRatePID   = rollRateController.compute(rollAnglePID,  gyroY, deltaTime);
        pitchRatePID  = pitchRateController.compute(pitchAnglePID, gyroX, deltaTime);
        yawRatePID    = yawRateController.compute(yawSpeed,       gyroZ, deltaTime);
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
        rollAnglePID  = rollAngleController.compute(desiredRoll,    roll,     deltaTime);
        pitchAnglePID = pitchAngleController.compute(desiredPitch,  pitch,    deltaTime);
        rollRatePID   = rollRateController.compute(rollAnglePID,    gyroY,    deltaTime);
        pitchRatePID  = pitchRateController.compute(pitchAnglePID,  gyroX,    deltaTime);
        yawRatePID    = yawRateController.compute(yawSpeed,         gyroZ,    deltaTime);
        altitudePID   = altitudeController.compute(desiredAltitude, altitude, deltaTime);
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
        rollAnglePID  = rollAngleController.compute(desiredRoll,   roll,          deltaTime);
        pitchAnglePID = pitchAngleController.compute(desiredPitch, pitch,         deltaTime);
        rollRatePID   = rollRateController.compute(rollAnglePID,   gyroY,         deltaTime);
        pitchRatePID  = pitchRateController.compute(pitchAnglePID, gyroX,         deltaTime);
        yawRatePID    = yawRateController.compute(yawSpeed,        gyroZ,         deltaTime);
        altitudePID   = altitudeController.compute(hoverAltitude,  altitude,      deltaTime);
        manualControlled = false;
        return true;
    }
}

void motorMixingAlgorithm(bool altitudeControlled)
{
    if (!altitudeControlled) {
        //if greater than 80% throttle clamp to 80%
        if (mainThrottleInput > MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC) {
            mainThrottleInput = MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC;
        }

        //if greater than 5% throttle use the PID controllers
        if (mainThrottleInput > MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC) {
            motorMatrix[0] = mainThrottleInput + pitchRatePID + rollRatePID  - yawRatePID;
            motorMatrix[1] = mainThrottleInput + pitchRatePID - rollRatePID  + yawRatePID;
            motorMatrix[2] = mainThrottleInput - pitchRatePID - rollRatePID  - yawRatePID;
            motorMatrix[3] = mainThrottleInput - pitchRatePID + rollRatePID  + yawRatePID;
            for (int i = 0; i < 4; i++)
            {
                //if after mixing the motors would be slower than 5%, then clamp the motors to 5%
                if (motorMatrix[i] < (MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC)) {
                    motorMatrix[i] = MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC;
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
            if (motorMatrix[i] < MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC && mainThrottleInput > MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC)
            {
                motorMatrix[i] = MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC;
            }
        }
    }
    else
    {
        anti_Windup();
        if (altitudePID > MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC)
        {
            altitudePID = MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC;
        }
        if (altitudePID >= (hoverThrottle)) {
            motorMatrix[0] = altitudePID + pitchRatePID + rollRatePID  - yawRatePID;
            motorMatrix[1] = altitudePID + pitchRatePID - rollRatePID  + yawRatePID;
            motorMatrix[2] = altitudePID - pitchRatePID - rollRatePID  - yawRatePID;
            motorMatrix[3] = altitudePID - pitchRatePID + rollRatePID  + yawRatePID;
        }

    }
}

void anti_Windup()
{
    if (altitudePID > MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC && altitudePID * altitudeController.error > 0)
    {
        altitudePID = altitudeController.unwind(deltaTime);
    }
}

void clampMixedMotors()
{
    for (int i = 0; i < 4; i++)
    {
        if (motorMatrix[i] > MAX_THROTTLE_VALUE) motorMatrix[i] = MAX_THROTTLE_VALUE;
        if (manualControlled && (motorMatrix[i] < MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC) && mainThrottleInput >= MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC) {
            motorMatrix[i] = MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC;
            // Serial.printf("Condition1\n");
        } 
        else if (motorMatrix[i] < MIN_THROTTLE_VALUE) {
            motorMatrix[i] = MIN_THROTTLE_VALUE;
            // Serial.printf("Condition2\n");
        }
    }
}
