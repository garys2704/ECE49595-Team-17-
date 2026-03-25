#include "led_debug.h"
#include "main.h"
#include "stm32f7xx_hal.h"

//static uint32_t lastBeat = 0;

void DebugLeds_Init(void)
{
  // Optional: ensure all off at boot
  HAL_GPIO_WritePin(DBG_LED1_GPIO_Port, DBG_LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DBG_LED_2_GPIO_Port, DBG_LED_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DBG_LED_3_GPIO_Port, DBG_LED_3_Pin,  GPIO_PIN_RESET);
}

static void pulse(GPIO_TypeDef* port, uint16_t pin, uint32_t ms)
{
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  HAL_Delay(ms);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

void DebugLeds_OnEvent(const UiEvent* e)
{
  if (!e) return;

  switch (e->type) {
    case UI_EVT_ENC_ROTATE:
      // quick pulse each detent event
      pulse(DBG_LED1_GPIO_Port, DBG_LED1_Pin, 10);
      break;

    case UI_EVT_ENC_CLICK:
      pulse(DBG_LED1_GPIO_Port, DBG_LED1_Pin, 80);
      break;

    case UI_EVT_LEFT:
      pulse(DBG_LED_2_GPIO_Port, DBG_LED_2_Pin, 80);
      break;

    case UI_EVT_RIGHT:
      pulse(DBG_LED_3_GPIO_Port, DBG_LED_3_Pin, 80);
      break;

    /*case UI_EVT_BACK:
    case UI_EVT_SELECT:
      // optional: reuse CLICK LED
      pulse(DBG_CLICK_GPIO_Port, DBG_CLICK_Pin, 120);
      break;

    case UI_EVT_LONGPRESS_LEFT:
    case UI_EVT_LONGPRESS_RIGHT:
      // long pulse = longpress visible
      pulse(DBG_CLICK_GPIO_Port, DBG_CLICK_Pin, 400);
      break;*/

    default:
      break;
  }
}

/*void DebugLeds_Heartbeat(void)
{
  uint32_t t = HAL_GetTick();
  if (t - lastBeat >= 500) {
    lastBeat = t;
    // If you want a dedicated heartbeat LED, use one of the debug LEDs:
    HAL_GPIO_TogglePin(DBG_ROT_GPIO_Port, DBG_ROT_Pin);
  }
}*/
