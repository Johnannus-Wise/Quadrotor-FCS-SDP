#include "utils.h"

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
