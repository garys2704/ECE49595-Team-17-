#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  UI_EVT_NONE = 0,
  UI_EVT_ENC_ROTATE,
  UI_EVT_ENC_CLICK,
  UI_EVT_ENC_LONGPRESS,
  UI_EVT_LEFT,
  UI_EVT_RIGHT,
  UI_EVT_BACK,
  UI_EVT_SELECT,
  UI_EVT_LONGPRESS_LEFT,
  UI_EVT_LONGPRESS_RIGHT
} UiEventType;

typedef struct {
  UiEventType type;
  int32_t value; // for rotations
} UiEvent;
