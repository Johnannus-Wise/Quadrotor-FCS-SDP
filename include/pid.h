#pragma once
#include "config.h"

// ============================================================
//  PIDController
//  Cascade-friendly PID with:
//    • derivative-on-measurement (no kick on setpoint change)
//    • first-order derivative low-pass filter
//    • conditional integration (dead-band tolerance)
//    • integral-unwind helper for anti-windup
// ============================================================

struct PIDController
{
    float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;
    float derivativeFilterAlpha = 0.95f;
    float tolerance             = 0.0f;

    float error                  = 0.0f;
    float integralTerm           = 0.0f;
    float derivativeTermRaw      = 0.0f;
    float derivativeTermFiltered = 0.0f;
    float previousFeedbackState  = 0.0f;
    float PID_output             = 0.0f;

    PIDController(float kp, float ki, float kd, float tol)
        : Kp(kp), Ki(ki), Kd(kd), tolerance(tol) {}

    float compute(float reference, float feedback, float dt)
    {
        if (dt <= 0.0f) return PID_output;

        error = reference - feedback;

        if (fabsf(error) > tolerance)
        {
            // Derivative on measurement — suppresses setpoint-change spikes
            derivativeTermRaw      = -(feedback - previousFeedbackState) / dt;
            derivativeTermFiltered = derivativeFilterAlpha * derivativeTermFiltered
                                   + (1.0f - derivativeFilterAlpha) * derivativeTermRaw;
            // Integral bounds — prevent windup in long-term error situations (e.g. takeoff). Currently set to +/- ~5% throttle.
            if (integralTerm <= 50.0f && integralTerm >= -50.0f) {
                integralTerm += error * dt;
            } 
            if (integralTerm > 50.0f) {
                integralTerm = 50.0f;
            } else if (integralTerm < -50.0f) {
                integralTerm = -50.0f;
            }
            

            PID_output = (Kp * error)
                       + (Ki * integralTerm)
                       + (Kd * derivativeTermFiltered);
        }
        previousFeedbackState = feedback;
        return PID_output;
    }

    void reset()
    {
        error                  = 0.0f;
        integralTerm           = 0.0f;
        derivativeTermRaw      = 0.0f;
        derivativeTermFiltered = 0.0f;
        PID_output             = 0.0f;
        previousFeedbackState  = 0.0f;
    }

    // Integral unwind step used by anti-windup
    float unwind(float dt)
    {
        if (dt <= 0.0f) return PID_output;
        integralTerm  -= error * dt;
        PID_output     = (Kp * error)
                       + (Ki * integralTerm)
                       + (Kd * derivativeTermFiltered);
        return PID_output;
    }
};
