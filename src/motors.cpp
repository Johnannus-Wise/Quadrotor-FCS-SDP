#include "motors.h"
#include "receiver.h"
#include "globals.h"

bool manualControlled = false;

int hoverThrottle = MAX_THROTTLE_VALUE * (1.8f / 4.0f);

// ============================================================
//  movement — called by the 500 Hz rate loop in main.cpp
// ============================================================

bool useIntegral() {
    return mainThrottleInput > (int)(MAX_THROTTLE_VALUE * INTEGRATION_GATE_FRAC);
}

void movement()
{
    updateOuterPIDLoop(useIntegral());
    updateRatePIDLoop(useIntegral());
    motorMixingAlgorithm(determineFlightControlMode());
    clampMixedMotors();
    ledcWrite(FRONT_LEFT,  motorMatrix[0]);
    ledcWrite(FRONT_RIGHT, motorMatrix[1]);
    ledcWrite(REAR_RIGHT,  motorMatrix[2]);
    ledcWrite(REAR_LEFT,   motorMatrix[3]);
}

// ============================================================
//  updatePID — 500 Hz
//  Runs rate controllers only. altitudePID is computed inside
//  motorMixingAlgorithm() at its own 50 Hz gate and persists
//  as a global between those ticks.
//  Returns true when altitude PID drives the throttle channel.
// ============================================================

void updateRatePIDLoop(bool integralActive)
{
    if (currentTime - attitudeLoopLast >= ATTITUDE_LOOP_US)
    {
        dtAttitude = (float)(currentTime - attitudeLoopLast) / 1e6f;  // convert to seconds
        attitudeLoopLast = currentTime;

        rollRatePID  = rollRateController.compute(rollAnglePID,  gyroY, dtAttitude);
        pitchRatePID = pitchRateController.compute(pitchAnglePID, gyroX, dtAttitude);
        yawRatePID   = yawRateController.compute(yawSpeed,       gyroZ, dtAttitude);
    }
}

void updateOuterPIDLoop(bool integralActive)
{
    if (currentTime - altitudeLoopLast >= ALTITUDE_LOOP_US)
    {
        dtAltitude = (float)(currentTime - altitudeLoopLast) / 1e6f;  // convert to seconds
        altitudeLoopLast = currentTime;

        float altSetpoint = altitudeLock ? hoverAltitude : (float)desiredAltitude;
        rollAnglePID  = rollAngleController.compute((float)desiredRoll,  accelRoll, dtAttitude);
        pitchAnglePID = pitchAngleController.compute((float)desiredPitch, accelPitch, dtAttitude);
        altitudePID = altitudeController.compute(altSetpoint, altitude, dtAltitude) + MIN_THROTTLE_VALUE;

        if (integralActive) {
            anti_Windup();
        }
    }
}

//Returns true if altitude PID is active and sets the the altitude set point for the altitude PID controller.
//Otherwise, false (manual throttle).

bool determineFlightControlMode()
{
    if (channels.ch[5].data < 500)
    {
        // ---- Angle Only (Manual Throttle) ----
        if (altitudeLock) {
            altitudeLock = false;
        }
        manualControlled = true;
        return false;
    }
    else if (channels.ch[5].data > 1400)
    {
        // ---- Angle + Altitude (Autonomous Throttle) ----
        if (altitudeLock) {
            altitudeLock = false;
        }
        manualControlled = false;
        return true;
    }
    else
    {
        // ---- Altitude Hold (Hover) ----
        if (!altitudeLock)
        {
            altitudeLock  = true;
        }
        manualControlled = false;
        return true;
    }
}

// ============================================================
//  motorMixingAlgorithm
//
//  Manual throttle path: mixes mainThrottleInput with rate PIDs
//  directly — no altitude PID involved.
//
//  Altitude-controlled path: the altitude PID gate fires at 50 Hz
//  (ALTITUDE_LOOP_US). On ticks where the gate fires, altitudePID
//  is recomputed and altitudeUpdateLast is stamped so dtAltitude
//  is always the true inter-compute interval. On the remaining
//  500 Hz ticks the last altitudePID value is reused for mixing,
//  keeping motor commands smooth at full rate.
// ============================================================

void motorMixingAlgorithm(bool isAltitudeControlled)
{
    if (!isAltitudeControlled) {
        // ── Manual throttle ─────────────────────────────────────────────────

        // Cap main throttle input to 80% throttle to preserve headroom for attitude corrections.
        if (mainThrottleInput > (int)(MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC)) {
            mainThrottleInput = (int)(MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC);
        }

        // Mix with control PID output if throttle is above minimum threshold of 5%.
        if (mainThrottleInput > (int)(MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC))
        {
            
            motorMatrix[0] = mainThrottleInput + rollRatePID + pitchRatePID - yawRatePID;
            motorMatrix[1] = mainThrottleInput - rollRatePID + pitchRatePID + yawRatePID;
            motorMatrix[2] = mainThrottleInput - rollRatePID - pitchRatePID - yawRatePID;
            motorMatrix[3] = mainThrottleInput + rollRatePID - pitchRatePID + yawRatePID;

            // for (int i = 0; i < 4; i++)
            // {
            //     if (motorMatrix[i] < (int)(MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC))
            //         motorMatrix[i] = (int)(MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC);
            // }
        }
        else
        {
            //use direct throttle pass-through if below threshold of 5% to avoid control corrections at very low throttle where they are not effective.
            motorMatrix[0] = mainThrottleInput;
            motorMatrix[1] = mainThrottleInput;
            motorMatrix[2] = mainThrottleInput;
            motorMatrix[3] = mainThrottleInput;
        }

        // Floor motors to minimum 10% throttle if mixing goes below that.
        for (int i = 0; i < 4; i++)
        {
            if (motorMatrix[i] < (int)(MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC) &&
                mainThrottleInput > (int)(MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC))
            {
                motorMatrix[i] = (int)(MAX_THROTTLE_VALUE * MIN_THROTTLE_FRAC);
            }
        }
    }
    else {
        // ── Altitude-controlled throttle ────────────────────────────────────

        // Mix at full 500 Hz using the most recent altitudePID value (alttiude PID is updated at 50 Hz).
        if ((altitudePID) > MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC) {
            altitudePID = MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC;
        }

        motorMatrix[0] = (int) altitudePID + rollRatePID + pitchRatePID - yawRatePID;
        motorMatrix[1] = (int) altitudePID - rollRatePID + pitchRatePID + yawRatePID;
        motorMatrix[2] = (int) altitudePID - rollRatePID - pitchRatePID - yawRatePID;
        motorMatrix[3] = (int) altitudePID + rollRatePID - pitchRatePID + yawRatePID;
    }
}

// ============================================================
//  anti_Windup — called inside the altitude PID gate only
// ============================================================

void anti_Windup()
{
    if (altitudePID > MAX_THROTTLE_VALUE * MAX_THROTTLE_FRAC &&
        altitudePID * altitudeController.error > 0)
    {
        altitudePID = altitudeController.unwind(dtAltitude);
    }
}

void clampMixedMotors()
{
    for (int i = 0; i < 4; i++)
    {
        if (motorMatrix[i] > MAX_THROTTLE_VALUE)
            motorMatrix[i] = MAX_THROTTLE_VALUE;
    }
}

void resetControllers() {
    rollAngleController.reset();
    pitchAngleController.reset();
    rollRateController.reset();
    pitchRateController.reset();
    yawRateController.reset();
    altitudeController.reset();
}