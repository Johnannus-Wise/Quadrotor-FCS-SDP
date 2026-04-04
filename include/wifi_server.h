#pragma once
#include "globals.h"
#include <WebServer.h>

extern WebServer server;

void PIDWebPage();
void handleRoot();
void handleUpdate();
void handleTelemetry();
