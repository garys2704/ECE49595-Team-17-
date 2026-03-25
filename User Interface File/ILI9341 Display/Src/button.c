#include "button.h"
#include "main.h"
#include "stm32f7xx_hal.h"

#define DEBOUNCE_MS 20

typedef struct {
  GPIO_TypeDef* port;
  uint16_t pin;
  bool last_raw;
  bool stable;
  uint32_t last_change;
} DebBtn;

static DebBtn leftb, rightb;

static bool read_released(DebBtn* b){
  return (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_SET); // pull-up -> released = SET
}

static bool debounce_update(DebBtn* b, bool* pressed_edge){
  *pressed_edge = false;

  bool raw_released = read_released(b);
  uint32_t t = HAL_GetTick();

  if (raw_released != b->last_raw) {
    b->last_raw = raw_released;
    b->last_change = t;
  }

  if ((t - b->last_change) >= DEBOUNCE_MS && raw_released != b->stable) {
    b->stable = raw_released;

    // edge; released -> pressed means stable became false
    if (!b->stable) *pressed_edge = true;
    return true;
  }

  return false;
}

void Buttons_Init(void){
  leftb.port = LEFT_BTN_GPIO_Port;
  leftb.pin  = LEFT_BTN_Pin;
  leftb.last_raw = leftb.stable = read_released(&leftb);
  leftb.last_change = HAL_GetTick();

  rightb.port = RIGHT_BTN_GPIO_Port;
  rightb.pin  = RIGHT_BTN_Pin;
  rightb.last_raw = rightb.stable = read_released(&rightb);
  rightb.last_change = HAL_GetTick();
}

bool Buttons_Poll(UiEvent* out){
  if (!out) return false;

  bool edge;

  if (debounce_update(&leftb, &edge) && edge) {
    out->type = UI_EVT_LEFT;
    out->value = 0;
    return true;
  }

  if (debounce_update(&rightb, &edge) && edge) {
    out->type = UI_EVT_RIGHT;
    out->value = 0;
    return true;
  }

  return false;
}
