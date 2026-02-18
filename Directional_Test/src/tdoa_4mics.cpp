#include <Arduino.h>

// !!! placeholder pins - replace with actual values !!!
const int micPin0 = A0; // Front
const int micPin1 = A1; // Right
const int micPin2 = A2; // Back
const int micPin3 = A3; // Left

const int motorPin0 = 3;
const int motorPin1 = 4;
const int motorPin2 = 5;
const int motorPin3 = 6;

// center/trigger values are the same as the old 'micCenterL'
const int micCenter0 = 5;
const int micCenter1 = 5;
const int micCenter2 = 5;
const int micCenter3 = 5;

const int triggerThreshold0 = 2;
const int triggerThreshold1 = 2;
const int triggerThreshold2 = 2;
const int triggerThreshold3 = 2;

static const uint32_t FS_HZ = 16000; // sample frequency
static const uint32_t TS_US = 1000000UL / FS_HZ; // sample period
static const int N = 128; // frame size
static const int LAG_MAX = 8; // temp, dependent on mic spacing
static const int MIN_TRIGGER_AMP = 5; // extra guard

static int16_t buf0[N];
static int16_t buf1[N];
static int16_t buf2[N];
static int16_t buf3[N];

// DC removal
static void removeMean(int16_t *x, int n) {
  int32_t sum = 0;
  for (int i = 0; i < n; i++) {
    sum += x[i];
  }
  int16_t mean = (int16_t)(sum / n);
  for (int i = 0; i < n; i++) {
    x[i] = (int16_t)(x[i] - mean);
  }
}

// light PHAT weighting (phase transform)
static void preEmphasis(int16_t *x, int n) {
  for (int i = n - 1; i >= 1; i--) x[i] = (int16_t)(x[i] - x[i - 1]);
  x[0] = 0;
}

// limited-lag correlation (returns best lag in samples)
// lag > 0 -> R arrives later than L (sound is closer to L)
// lag < 0 -> R arrives earlier than L (sound is closer to R)
static int lagCorrelation(const int16_t *Lsig, const int16_t *Rsig, int n, int lagMax) {
  int bestLag = 0; // lag with the highest correlation score
  int64_t bestScore = INT64_MIN;

  for (int lag = -lagMax; lag <= lagMax; lag++) {
    int startL = 0; // Lsig start index
    int startR = 0; // Rsig start index
    int count = n; // number of overlapping samples

    if (lag > 0) { // shift Rsig fwd, skip first 'lag' samples of Rsig
      startR = lag; 
      count = n - lag;
    }
    if (lag < 0) { // shift Lsig fwd, skip first '-lag' samples of Lsig
      startL = -lag; 
      count = n + lag;
    }

    int64_t acc = 0; // accumlated similarity score

    // amplifies aligned samples to see correlation
    for (int i = 0; i < count; i++) {
      acc += (int32_t)Lsig[startL + i] * (int32_t)Rsig[startR + i];
    }

    if (acc > bestScore) {
      bestScore = acc;
      bestLag = lag;
    }
  }
  return bestLag;
}

// capture N samples w/ timed polling
static void captureFrame(int16_t *m0, int16_t *m1, int16_t *m2, int16_t *m3) {
  uint32_t next = micros();

  for (int i = 0; i < N; i++) {
    // sequential not simultaneous (will need updated)
    int raw0 = analogRead(micPin0);
    int raw1 = analogRead(micPin1);
    int raw2 = analogRead(micPin2);
    int raw3 = analogRead(micPin3);

    m0[i] = (int16_t)(raw0 - micCenter0);
    m1[i] = (int16_t)(raw1 - micCenter1);
    m2[i] = (int16_t)(raw2 - micCenter2);
    m3[i] = (int16_t)(raw3 - micCenter3);

    next += TS_US;
    while ((int32_t)(micros() - next) < 0) {
      // wait until next sample time
    }
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);

  pinMode(motorPin0, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);

  analogWrite(motorPin0, 0);
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);

  Serial.println("System Initialized - STM32F767ZI");
}

void loop() {
  // 1. Read Sensors
  int sensorValue0 = analogRead(micPin0); // Front
  int sensorValue1 = analogRead(micPin1); // Right
  int sensorValue2 = analogRead(micPin2); // Back
  int sensorValue3 = analogRead(micPin3); // Left

  // calculate Amplitude (Distance from "Quiet" voltage)
  int amplitude0 = abs(sensorValue0 - micCenter0);
  int amplitude1 = abs(sensorValue1 - micCenter1);
  int amplitude2 = abs(sensorValue2 - micCenter2);
  int amplitude3 = abs(sensorValue3 - micCenter3);

  // define Loudness Range
  int minLoudness0 = 0; int maxLoudness0 = 10;
  int minLoudness1 = 0; int maxLoudness1 = 10;
  int minLoudness2 = 0; int maxLoudness2 = 10;
  int minLoudness3 = 0; int maxLoudness3 = 10;

  // map Amplitude to PWM (0-255) (scales amplitude to motor speed)
  int vibrationIntensity0 = map(amplitude0, minLoudness0, maxLoudness0, 0, 255);
  int vibrationIntensity1 = map(amplitude1, minLoudness1, maxLoudness1, 0, 255);
  int vibrationIntensity2 = map(amplitude2, minLoudness2, maxLoudness2, 0, 255);
  int vibrationIntensity3 = map(amplitude3, minLoudness3, maxLoudness3, 0, 255);

  // constrain (Safety Clipping) (no negative values to motors)
  vibrationIntensity0 = constrain(vibrationIntensity0, 0, 255);
  vibrationIntensity1 = constrain(vibrationIntensity1, 0, 255);
  vibrationIntensity2 = constrain(vibrationIntensity2, 0, 255);
  vibrationIntensity3 = constrain(vibrationIntensity3, 0, 255);

  // Serial monitor formatting for prototyping
  
  // I hid this because i didnt feel like formatting it but feel free to if you need to see the values

  /* Serial.print("RawR: "); Serial.print(sensorValueR);
  Serial.print(" | RawL: "); Serial.print(sensorValueL);
  Serial.print(" | AmpR: "); Serial.print(amplitudeR);
  Serial.print(" | AmpL: "); Serial.println(amplitudeL);*/

  // 2. Trigger condition
  int maxAmp = max(max(amplitude0, amplitude1), max(amplitude2, amplitude3));
  bool triggered = (amplitude0 > triggerThreshold0 || amplitude1 > triggerThreshold1
                  || amplitude2 > triggerThreshold2 || amplitude3 > triggerThreshold3) 
                  && (maxAmp > MIN_TRIGGER_AMP);

  if (triggered) {
    // 3. Capture a short frame and compute TDOA
    captureFrame(buf0, buf1, buf2, buf3);

    // preprocess for correlation stab.
    removeMean(buf0, N);
    removeMean(buf1, N);
    removeMean(buf2, N);
    removeMean(buf3, N);

    preEmphasis(buf0, N);
    preEmphasis(buf1, N);
    preEmphasis(buf2, N);
    preEmphasis(buf3, N);

    int lagX = lagCorrelation(buf3, buf1, N, LAG_MAX); // Left vs Right
    int lagY = lagCorrelation(buf0, buf2, N, LAG_MAX); // Front vs Back

    float dtX_us = (1e6f * (float)lagX) / (float)FS_HZ;
    float dtY_us = (1e6f * (float)lagY) / (float)FS_HZ;

    // 4. Decide direction from lag sign
    // turn on/off corresponding motor pin
    // 0, 1, 2, 3 = FRONT, RIGHT, BACK, LEFT
    if (abs(lagX) > abs(lagY)) { // LEFT RIGHT (1, 3)
      if (lagX > 0) { // vibrate left
        Serial.print(">>> LEFT by TDOA, lag="); Serial.print(lagX);
        Serial.print(" samples, dt="); Serial.print(dt_us); Serial.println(" us");
        analogWrite(motorPin3, vibrationIntensity3);
        analogWrite(motorPin1, 0);
        analogWrite(motorPin0, 0);
        analogWrite(motorPin2, 0);
      }
      else if (lagX < 0) { // vibrate right
        Serial.print(">>> RIGHT by TDOA, lag="); Serial.print(lagX);
        Serial.print(" samples, dt="); Serial.print(dt_us); Serial.println(" us");
        analogWrite(motorPin1, vibrationIntensity1);
        analogWrite(motorPin3, 0);
        analogWrite(motorPin0, 0);
        analogWrite(motorPin2, 0);
      }
      else {
        Serial.println(">>> CENTER L/R (lagX=0)");
        analogWrite(motorPin0, 0);
        analogWrite(motorPin1, vibrationIntensity1 / 4);
        analogWrite(motorPin2, 0);
        analogWrite(motorPin3, vibrationIntensity3 / 4);
      }
    } 
    else if (abs(lagX) < abs(lagY)) { // FRONT BACK (0, 2)
      if (lagY > 0) { // vibrate front
        Serial.print(">>> FRONT by TDOA, lag="); Serial.print(lagY);
        Serial.print(" samples, dt="); Serial.print(dt_us); Serial.println(" us");
        analogWrite(motorPin0, vibrationIntensity0);
        analogWrite(motorPin2, 0);
        analogWrite(motorPin1, 0);
        analogWrite(motorPin3, 0);
      }
      else if (lagY < 0) { // vibrate back
        Serial.print(">>> BACK by TDOA, lag="); Serial.print(lagY);
        Serial.print(" samples, dt="); Serial.print(dt_us); Serial.println(" us");
        analogWrite(motorPin2, vibrationIntensity2);
        analogWrite(motorPin0, 0);
        analogWrite(motorPin1, 0);
        analogWrite(motorPin3, 0);
      }
      else {
        Serial.println(">>> CENTER F/B (lagY=0)");
        analogWrite(motorPin0, vibrationIntensity0 / 4);
        analogWrite(motorPin1, 0);
        analogWrite(motorPin2, vibrationIntensity2 / 4);
        analogWrite(motorPin3, 0);
      }
    } 
    else { // split vibration (lag = 0)
      Serial.println(">>> CENTER (lag=0)");
      analogWrite(motorPin0, vibrationIntensity0 / 4);
      analogWrite(motorPin1, vibrationIntensity1 / 4);
      analogWrite(motorPin2, vibrationIntensity2 / 4);
      analogWrite(motorPin3, vibrationIntensity3 / 4);
    }

    // delay so user can feel it
    delay(40);

    // set motor pins back to 0
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);

  } 
  else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
  }

  delay(5);
}
