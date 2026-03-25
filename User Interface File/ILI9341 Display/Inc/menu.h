#pragma once
#include <stdbool.h>
#include "ui_event.h"

void Menu_Init(void);
void Menu_HandleEvent(const UiEvent* ev);
void Menu_Render(void);

bool Menu_IsDirty(void);
void Menu_ClearDirty(void);
