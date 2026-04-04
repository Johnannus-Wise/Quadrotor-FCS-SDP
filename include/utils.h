#pragma once
#include "config.h"
// ============================================================
//  Utility / math helpers
//  Previously defined as file-scope functions inside main.cpp.
//  Extracting them here makes them testable and prevents
//  accidental multiple-definition if main.cpp is ever split.
// ============================================================

// Linear mapping helpers used by Channels::setDutyCycle and rescale()
float calSlope(float y2, float y1, float x2, float x1);
float calYIntercept(float slope, float x, float y);
