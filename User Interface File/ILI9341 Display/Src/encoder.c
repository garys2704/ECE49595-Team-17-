#include "stm32f7xx_hal.h"
#include "encoder.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "ui_event.h"

#define ENC_TIM htim3

#define COUNTS_PER_DETENT 2

// button debounce timing -- will get button functionality later
#define DEBOUNCE_MS 20
#define LONGPRESS_MS 1000

static int32_t prev = 0;
static int32_t accum = 0;
static uint32_t btnLastChange_ms = 0;
static uint32_t btnPressStart_ms = 0;
static bool btnStableState = true;
static bool btnLastReport = true;

int32_t getCount(void) {
    return (int32_t)__HAL_TIM_GET_COUNTER(&ENC_TIM);
}

static inline uint32_t getms(void) {
	return HAL_GetTick();
}

// get timer count
void encoderInit(void) {
	HAL_TIM_Encoder_Start(&ENC_TIM, TIM_CHANNEL_ALL);
	prev = getCount();
	btnLastReport = (HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin) == GPIO_PIN_SET); // released
	btnStableState = btnLastReport;
	btnLastChange_ms = getms();
}

static bool btnSample(bool* outClick, bool* outLongpress) {
  *outClick = false;
  *outLongpress = false;

  bool rawRelease = (HAL_GPIO_ReadPin(ENC_BTN_GPIO_Port, ENC_BTN_Pin) == GPIO_PIN_SET);
  uint32_t t = getms();

  // debounce & detect changes in raw input
  if (rawRelease != btnLastReport) {
    btnLastReport = rawRelease;
    btnLastChange_ms = t;
  }

  // if stable long enough, accept new stable state
  if ((t - btnLastChange_ms) >= DEBOUNCE_MS && rawRelease != btnStableState) {
    btnStableState = rawRelease;

    // transition from released -> pressed
    if (!btnStableState) {
      btnPressStart_ms = t;
    } else {
      // transition from pressed -> released
      uint32_t held = t - btnPressStart_ms;
      if (held >= LONGPRESS_MS) {
        *outLongpress = true;
      } else {
        *outClick = true;
      }
    }
    return true;
  }

  return false;
}

// gets the raw total movement of the encoder
int32_t getDelta(void){
	int32_t now = getCount();
	int32_t delta = now - prev;

	// wrapping
	if (delta > 32767) delta -= 65536;
	if (delta < -32768) delta += 65536;

	prev = now;
	return delta;
}

// converts raw encoder movement into detents
int32_t getDetents(void){
	int32_t delta = getDelta();
	accum += delta;

	int32_t detents = accum / COUNTS_PER_DETENT;
	accum -= detents * COUNTS_PER_DETENT;
	return detents;
}

// periodically check the encoder for new events
bool encoderCheck(EncEvent* out){
	if (!out) return false;

	// logic for turns
	int32_t detents = getDetents();
	if (detents != 0) {
		out->type = ENC_EVT_ROTATE;
		out->steps = detents;
		return true;
	}

	// logic for button press
	bool outClick = false;
	bool outPress = false;
	bool btnChange = btnSample(&outClick, &outPress);
	if (btnChange) {
		if (outClick){
			out->type = ENC_EVT_CLICK;
			out->steps = 0;
			return true;
		}
		else if (outPress){
			out->type = ENC_EVT_LONGPRESS;
			out->steps = 0;
			return true;
		}
	}
	return false;
}

bool Encoder_PollUi(UiEvent* out)
{
  EncEvent ev;
  if (!out) return false;
  if (!encoderCheck(&ev)) return false;

  out->value = 0;
  switch (ev.type) {
    case ENC_EVT_ROTATE:
    	out->type = UI_EVT_ENC_ROTATE;
    	out->value = ev.steps;
    	return true;
    case ENC_EVT_CLICK:
    	out->type = UI_EVT_ENC_CLICK;
    	return true;
    case ENC_EVT_LONGPRESS:
		out->type = UI_EVT_ENC_LONGPRESS;
		return true;
    default:
    	return false;
  }
}
