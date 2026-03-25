#include "menu.h"
#include <stdio.h>
#include <string.h>

static void LCD_Clear(void) {}
static void LCD_SetCursor(int col, int row) { (void)col; (void)row; }
static void LCD_Print(const char* s) { (void)s; }

typedef enum {
    MENU_NAV = 0,
    MENU_EDIT
} MenuMode;

typedef struct {
  const char* name;
  int value;
  int min;
  int max;
  int step;
} Setting;

static Setting settings[] = {
  {"Volume", 50, 0, 100, 1},
  {"Brightness", 7, 0, 10, 1},
  {"Mode", 0, 0, 3, 1},
  // possible mic display
};

static const int numSettings = sizeof(settings) / sizeof(settings[0]);

static MenuMode mode = MENU_NAV;
static int selected = 0;
static bool dirty = true;

static int limit(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void Menu_Init(void) {
    mode = MENU_NAV;
    selected = 0;
    dirty = true;
}

bool Menu_IsDirty(void) {
	return dirty;
}

void Menu_ClearDirty(void) {
	dirty = false;
}

static void nav(int delta) {
  if (delta == 0) return; // no change
  sel = limit(sel + delta, 0, numSettings - 1);
  dirty = true; // prompt refresh
}

static void edit(int delta) {
  if (delta == 0) return; // no change

  Setting* setting = &settings[sel];
  int newVal = setting->value + delta * setting->step;
  newVal = limit(newVal, setting->min, setting->max);

  if (newVal != setting->value) {
    setting->value = newVal;
    dirty = true; // prompt refresh
  }
}

void Menu_HandleEvent(const UiEvent* ev) {
  if (!ev) return;

  switch (ev->type) {
    case UI_EVT_ENC_CLICK:
    case UI_EVT_SELECT:
      mode = (mode == UI_NAV) ? UI_EDIT : UI_NAV;
      dirty = true;
      break;

    case UI_EVT_ENC_ROTATE:
      if (mode == UI_NAV) nav((int)ev->value); // value = steps
      else edit((int)ev->value);
      break;

    case UI_EVT_LEFT:
      if (mode == UI_NAV) nav(-1);
      else edit(-1);
      break;

    case UI_EVT_RIGHT:
      if (mode == UI_NAV) nav(+1);
      else edit(+1);
      break;

    /*case UI_EVT_END_LONGPRESS:
      // Typical behavior: exit edit mode
      if (mode == UI_EDIT) {
        mode = UI_NAV;
        dirty = true;
      }
      break;

    case UI_EVT_LONGPRESS_LEFT:
    case UI_EVT_LONGPRESS_RIGHT:
      break;*/

    default:
      break;
  }
}

void Menu_Render(void)
{
  if (!dirty) return;

  char line1[17];
  char line2[17];

  const Setting* s = &settings[sel];

  // Line 1: selector + name
  // '>' in NAV, '*' in EDIT
  snprintf(line1, sizeof(line1), "%c %-14s",
           (mode == UI_NAV ? '>' : '*'),
           s->name);

  // Line 2: value + mode hint
  snprintf(line2, sizeof(line2), "Val:%4d %s",
           s->value,
           (mode == UI_EDIT ? "EDIT" : "    "));

  LCD_Clear();
  LCD_SetCursor(0, 0); LCD_Print(line1);
  LCD_SetCursor(0, 1); LCD_Print(line2);

  dirty = false;
}
