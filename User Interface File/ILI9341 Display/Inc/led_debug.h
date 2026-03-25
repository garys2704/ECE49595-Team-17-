#pragma once
#include "ui_event.h"

void DebugLeds_Init(void);
void DebugLeds_OnEvent(const UiEvent* e);
void DebugLeds_Heartbeat(void);   // call periodically
