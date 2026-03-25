#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <ui_event.h>

typedef enum {
  ENC_EVT_NONE = 0,
  ENC_EVT_ROTATE,
  ENC_EVT_CLICK,
  ENC_EVT_LONGPRESS
} EncEventType;

typedef struct {
  EncEventType type;
  int32_t steps; // detents for rotations
} EncEvent;

// call once after MX init
void encoderInit(void);

// call periodically (5–20ms), returns true if an event produced
bool encoderCheck(EncEvent* out);

int32_t getDetents(void); // returns detents since last call

bool Encoder_PollUi(UiEvent* out);
