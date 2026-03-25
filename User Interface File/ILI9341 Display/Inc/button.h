#pragma once
#include <stdbool.h>
#include "ui_event.h"

void Buttons_Init(void);
bool Buttons_Poll(UiEvent* out);   // call every 5–20ms
