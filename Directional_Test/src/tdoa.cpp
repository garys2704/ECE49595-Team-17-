#include <Arduino.h>

const int micPinR = A2;
const int motorPinR = 6;
const int micPinL = A3;
const int motorPinL = 5;

const int micCenterR = 99;
const int micCenterL = 5;

const int triggerThresholdR = 100;
const int triggerThresholdL = 2;

static const uint32_t FS_HZ = 16000; // sample frequency
static const uint32_t TS_US = 1000000UL / FS_HZ; // sample period
static const int N = 128; // frame size
static const int LAG_MAX = 8; // temp, dependent on mic spacing
static const int MIN_TRIGGER_AMP = 5; // extra guard

static int16_t bufL[N];
static int16_t bufR[N];

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
static void captureFrame(int16_t *Lout, int16_t *Rout) {
  uint32_t next = micros();

  for (int i = 0; i < N; i++) {
    // sequential not simultaneous (will need updated)
    int rawL = analogRead(micPinL);
    int rawR = analogRead(micPinR);

    Lout[i] = (int16_t)(rawL - micCenterL);
    Rout[i] = (int16_t)(rawR - micCenterR);

    next += TS_US;
    while ((int32_t)(micros() - next) < 0) {
      // wait until next sample time
    }
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);

  pinMode(motorPinL, OUTPUT);
  pinMode(motorPinR, OUTPUT);
  analogWrite(motorPinR, 0);
  analogWrite(motorPinL, 0);

  Serial.println("System Initialized - STM32F767ZI");
}

void loop() {
  // 1. Read Sensors
  int sensorValueR = analogRead(micPinR);
  int sensorValueL = analogRead(micPinL);

  // calculate Amplitude (Distance from "Quiet" voltage)
  int amplitudeR = abs(sensorValueR - micCenterR);
  int amplitudeL = abs(sensorValueL - micCenterL);

  // define Loudness Range
  int minLoudnessR = 0;  
  int maxLoudnessR = 200;
  int minLoudnessL = 0;   
  int maxLoudnessL = 10;

  // map Amplitude to PWM (0-255) (scales amplitude to motor speed)
  int vibrationIntensityR = map(amplitudeR, minLoudnessR, maxLoudnessR, 0, 255);
  int vibrationIntensityL = map(amplitudeL, minLoudnessL, maxLoudnessL, 0, 255);

  // constrain (Safety Clipping) (no negative values to motors)
  vibrationIntensityR = constrain(vibrationIntensityR, 0, 255);
  vibrationIntensityL = constrain(vibrationIntensityL, 0, 255);

  // Serial monitor formatting for prototyping
  Serial.print("RawR: "); Serial.print(sensorValueR);
  Serial.print(" | RawL: "); Serial.print(sensorValueL);
  Serial.print(" | AmpR: "); Serial.print(amplitudeR);
  Serial.print(" | AmpL: "); Serial.println(amplitudeL);

  // 2. Trigger condition
  bool triggered = (amplitudeL > triggerThresholdL || amplitudeR > triggerThresholdR) && (max(amplitudeL, amplitudeR) > MIN_TRIGGER_AMP);

  if (triggered) {
    // 3. Capture a short frame and compute TDOA
    captureFrame(bufL, bufR);

    // preprocess for correlation stab.
    removeMean(bufL, N);
    removeMean(bufR, N);
    preEmphasis(bufL, N);
    preEmphasis(bufR, N);

    int lag = lagCorrelation(bufL, bufR, N, LAG_MAX);
    float dt_us = (1e6f * (float)lag) / (float)FS_HZ; // delta time, in microsecodns

    // 4. Decide direction from lag sign
    // turn on/off corresponding motor pin
    if (lag > 0) { // vibrate left
      Serial.print(">>> LEFT by TDOA, lag="); Serial.print(lag);
      Serial.print(" samples, dt="); Serial.print(dt_us); Serial.println(" us");
      analogWrite(motorPinL, vibrationIntensityL);
      analogWrite(motorPinR, 0);
    } 
    else if (lag < 0) { // vibrate right
      Serial.print(">>> RIGHT by TDOA, lag="); Serial.print(lag);
      Serial.print(" samples, dt="); Serial.print(dt_us); Serial.println(" us");
      analogWrite(motorPinR, vibrationIntensityR);
      analogWrite(motorPinL, 0);
    } 
    else { // split vibration (lag = 0)
      Serial.println(">>> CENTER (lag=0)");
      analogWrite(motorPinR, vibrationIntensityR / 2);
      analogWrite(motorPinL, vibrationIntensityL / 2);
    }

    // delay so user can feel it
    delay(40);

    // set both motor pins back to 0
    analogWrite(motorPinR, 0);
    analogWrite(motorPinL, 0);

  } 
  else {
    analogWrite(motorPinR, 0);
    analogWrite(motorPinL, 0);
  }

  delay(5);
}
