#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define TFT_CS PB6 //QSPI first right
#define TFT_RST PB1 //first ADC_IN left
#define TFT_DC PC0 //A1 left
#define TFT_MOSI PA7 //lower COMP left
#define TFT_CLK PA5 //first I2S right
//LED
//#define TFT_MISO PA6 //second I2S right

#define Right_Arrow PF10 //A5 left
#define Left_Arrow PF5 //A4 left
#define turnDet PE3 //I2C 3 right
//#define Rotary_Right
//#define Rotary_Left
int volume = 20;
int sensi = 40;
int test = 10;
int* stat = nullptr;
int oldStat = 0;
int menuIndex = 0;
int menuVal = 0;
int lastMenuIndex = 0;
int turn_enable = 1;
int turnDir = 1;
int menuFrames[] = {0,1};
int* battery;

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

void setup() {
  delay(2000);

  pinMode(TFT_CS, INPUT);
  pinMode(TFT_DC, INPUT);
  pinMode(TFT_RST, INPUT);

  // put your setup code here, to run once:
  pinMode(Right_Arrow, INPUT_PULLUP);
  pinMode(Left_Arrow, INPUT_PULLUP);
  pinMode(turnDet, INPUT_PULLUP);
  //pinMode()

  delay(10);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TFT_DC, HIGH);
  digitalWrite(TFT_RST, HIGH);

  SPI.setSCLK(PA5);
  SPI.setMOSI(PA7);
  SPI.begin();

  Serial3.begin(115200); //9600);
  delay(1000);
  Serial3.println("ILI9341 Test!");

  digitalWrite(TFT_RST, LOW);
  delay(20);
  digitalWrite(TFT_RST, HIGH);
  delay(150); 
 
  tft.setSPISpeed(8000000);
  tft.begin();

  tft.setRotation(2);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(3);
  tft.println("Welcome!");


  //stat = &volume;
  //oldStat = *stat;
  menuIndex = -1;
  lastMenuIndex = -1;
}
/*
struct MenuFrame
{
  int* stat;
  struct MenuFrame* next;
}
MenuFrame *newFrame(int* stat)
{
  struct MenuFrame *temp = new MenuFrame;
  temp -> stat = stat;
  temp -> next = NULL;
  return temp;
}
*/
int* setStat(int menuVal) {
  if(menuVal == 0){
    return &volume;
  }
  else if(menuVal == 1){
    return &sensi;
  }
  else return nullptr;
  /*
  else if (menuVal == 1) {
    return &sensi;
  }
  else {
    return &test;
  }
  */
}

unsigned long modifyStat(int* stat, int turnDir) {
  unsigned long start;
  if((turnDir == 1) && (*stat < 200)) {
    (*stat)++;
  }
  else if (*stat > 0) {
    (*stat)--;
  }
  else {
    *stat = *stat;
  }
  return micros() - start;
}

unsigned long MenuBar(int* stat) {
  unsigned long start;
  Serial3.println(*stat);
  int cx = tft.width()/2;
  int base = tft.height() - 50;

  tft.drawRect(cx - 10, base - 200, 20, 200, ILI9341_GREEN);

  if(*stat > oldStat){
    tft.fillRect(cx - 10, base - *stat, 20, *stat - oldStat, ILI9341_GREEN);
  }
  else {
    tft.fillRect(cx - 10, base - oldStat, 20, oldStat - *stat, ILI9341_BLACK);
  }
  tft.drawRect(cx - 10, base - 200, 20, 200, ILI9341_GREEN);
  oldStat = *stat;
  /*
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(ILI9341_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }
  */
  return micros() - start;
}

unsigned long MenuText(int menuVal) {
  tft.fillRect(0, 0, tft.width(), 50, ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE); tft.setTextSize(3);

  if(menuVal == 0){
    tft.println("Volume");
  }

  else if(menuVal == 1){
    tft.println("Sensitivity");
  }
  else{
  
  }
  /*
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  */
  return micros() - start;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(Right_Arrow) == LOW) {
    menuIndex++;
  }
  else if(digitalRead(Left_Arrow) == LOW) {
    menuIndex--;
  }
  if((digitalRead(turnDet) == LOW) && turn_enable){
    modifyStat(stat, turnDir);
    if (!((stat == nullptr)|(stat == battery))){
      MenuBar(stat);
    }
  }
  if(menuIndex != lastMenuIndex) {
    menuVal = menuFrames[abs(menuIndex % 2)];
    stat = setStat(menuVal);
    Serial3.println(menuIndex);
    Serial3.println(menuVal);
    Serial3.println(menuFrames[menuIndex % 2]);
    MenuText(menuVal);
    delay(50);
    MenuBar(stat);
    delay(50);
    
    lastMenuIndex = menuIndex;
  }
  delay(30);
}


