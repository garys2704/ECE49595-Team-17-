#include <Arduino.h>

// ─── Pin assignments (update to match your wiring) ───
const int micPin0 = A0; // Front
const int micPin1 = A1; // Right
const int micPin2 = A2; // Back
const int micPin3 = A3; // Left

const int motorPin0 = 3;  // Front
const int motorPin1 = 5;  // Right
const int motorPin2 = 6;  // Back
const int motorPin3 = 9;  // Left

// ─── ADC mid-rail bias ───
// With 12-bit ADC (0-4095), a typical electret/MEMS mic biased at VCC/2
// sits near 2048. These get auto-calibrated in setup().
static int micCenter0 = 2048;
static int micCenter1 = 2048;
static int micCenter2 = 2048;
static int micCenter3 = 2048;

// ─── Per-mic gain compensation ───
// Increase for weaker mics. Measure RMS from equal distance to find ratio.
// Example: if mic2 reads ~60% of the others, set its gain to ~1.7
static const float micGain[4] = {1.0f, 1.0f, 1.0f, 1.0f};

// ─── Sampling parameters ───
// 4 sequential analogReads on STM32F7 at ~15µs each ≈ 60µs total.
// 8 kHz is conservative and reliable. Increase only if you move to
// DMA/timer-driven ADC for true simultaneous sampling.
static const uint32_t FS_HZ    = 8000;
static const uint32_t TS_US    = 1000000UL / FS_HZ;
static const int      N        = 128;   // frame size (16ms at 8kHz)
static const int      LAG_MAX  = 8;     // max lag to search (samples)

// ─── Trigger thresholds ───
// Compared against frame RMS (not raw amplitude).
// 12-bit ADC: quiet room RMS ≈ 20-60 counts; speech ≈ 120-400.
static const int TRIGGER_RMS    = 60;
static const int MIN_PEAK_AMP   = 120;

// ─── Motor output ───
static const int MOTOR_MIN_PWM  = 80;   // minimum PWM to spin motor
static const int MOTOR_MAX_PWM  = 255;
static const int MOTOR_PULSE_MS = 60;   // vibration pulse duration (ms)

// ─── Per-mic noise floors ───
static int noiseFloor[4] = {32, 32, 32, 32};

static void updateNoiseFloor(int idx, int amp) {
  if (amp < 80) {
    noiseFloor[idx] = (noiseFloor[idx] * 31 + amp) / 32;
  }
}

// ─── Buffers ───
static int16_t buf0[N];
static int16_t buf1[N];
static int16_t buf2[N];
static int16_t buf3[N];

// ─── DC removal (subtract frame mean) ───
static void removeMean(int16_t *x, int n) {
  int32_t sum = 0;
  for (int i = 0; i < n; i++) sum += x[i];
  int16_t mean = (int16_t)(sum / n);
  for (int i = 0; i < n; i++) x[i] -= mean;
}

// ─── RMS energy of a frame ───
static int frameRMS(const int16_t *x, int n) {
  int64_t sumSq = 0;
  for (int i = 0; i < n; i++) {
    sumSq += (int32_t)x[i] * (int32_t)x[i];
  }
  return (int)sqrtf((float)sumSq / (float)n);
}

// ─── Peak amplitude of a frame ───
static int framePeak(const int16_t *x, int n) {
  int peak = 0;
  for (int i = 0; i < n; i++) {
    int a = abs(x[i]);
    if (a > peak) peak = a;
  }
  return peak;
}

// ─── Pre-emphasis (approximates PHAT weighting) ───
static void preEmphasis(int16_t *x, int n) {
  for (int i = n - 1; i >= 1; i--) {
    x[i] = (int16_t)(x[i] - x[i - 1]);
  }
  x[0] = 0;
}

// ─── Cross-correlation with limited lag ───
// Returns best lag in samples.
//   lag > 0  →  Rsig arrives later (sound closer to L)
//   lag < 0  →  Rsig arrives earlier (sound closer to R)
static int lagCorrelation(const int16_t *Lsig, const int16_t *Rsig,
                          int n, int lagMax) {
  int bestLag = 0;
  int64_t bestScore = INT64_MIN;

  for (int lag = -lagMax; lag <= lagMax; lag++) {
    int startL = (lag < 0) ? -lag : 0;
    int startR = (lag > 0) ?  lag : 0;
    int count  = n - abs(lag);

    int64_t acc = 0;
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

// ─── Capture N samples with timed polling ───
static void captureFrame(int16_t *m0, int16_t *m1, int16_t *m2, int16_t *m3) {
  uint32_t next = micros();

  for (int i = 0; i < N; i++) {
    m0[i] = (int16_t)((float)(analogRead(micPin0) - micCenter0) * micGain[0]);
    m1[i] = (int16_t)((float)(analogRead(micPin1) - micCenter1) * micGain[1]);
    m2[i] = (int16_t)((float)(analogRead(micPin2) - micCenter2) * micGain[2]);
    m3[i] = (int16_t)((float)(analogRead(micPin3) - micCenter3) * micGain[3]);

    next += TS_US;
    while ((int32_t)(micros() - next) < 0) { /* spin */ }
  }
}

// ─── Auto-calibrate mic DC bias at startup ───
static void calibrateMicBias() {
  Serial.println("Calibrating mic bias... keep environment quiet.");
  delay(500);

  const int CAL_SAMPLES = 256;
  int32_t sum0 = 0, sum1 = 0, sum2 = 0, sum3 = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    sum0 += analogRead(micPin0);
    sum1 += analogRead(micPin1);
    sum2 += analogRead(micPin2);
    sum3 += analogRead(micPin3);
    delayMicroseconds(500);
  }

  micCenter0 = (int)(sum0 / CAL_SAMPLES);
  micCenter1 = (int)(sum1 / CAL_SAMPLES);
  micCenter2 = (int)(sum2 / CAL_SAMPLES);
  micCenter3 = (int)(sum3 / CAL_SAMPLES);

  Serial.print("Mic bias: ");
  Serial.print(micCenter0); Serial.print(", ");
  Serial.print(micCenter1); Serial.print(", ");
  Serial.print(micCenter2); Serial.print(", ");
  Serial.println(micCenter3);
}

// ─── Map RMS to motor PWM ───
static int rmsToMotorPWM(int rms) {
  if (rms < TRIGGER_RMS) return 0;
  int pwm = map(rms, TRIGGER_RMS, 600, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
  return constrain(pwm, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
}

// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // 0-4095

  pinMode(motorPin0, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);

  analogWrite(motorPin0, 0);
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);

  calibrateMicBias();

  Serial.println("System Initialized - STM32F767ZI");
  Serial.print("Fs = "); Serial.print(FS_HZ);
  Serial.print(" Hz, Ts = "); Serial.print(TS_US);
  Serial.print(" us, N = "); Serial.println(N);
}

// ═══════════════════════════════════════════════════════════════
void loop() {
  // ── 1. Quick single-sample check ──
  int raw0 = analogRead(micPin0);
  int raw1 = analogRead(micPin1);
  int raw2 = analogRead(micPin2);
  int raw3 = analogRead(micPin3);

  int amp0 = abs((int)((float)(raw0 - micCenter0) * micGain[0]));
  int amp1 = abs((int)((float)(raw1 - micCenter1) * micGain[1]));
  int amp2 = abs((int)((float)(raw2 - micCenter2) * micGain[2]));
  int amp3 = abs((int)((float)(raw3 - micCenter3) * micGain[3]));

  updateNoiseFloor(0, amp0);
  updateNoiseFloor(1, amp1);
  updateNoiseFloor(2, amp2);
  updateNoiseFloor(3, amp3);

  // Quick trigger: any mic above its noise floor + margin?
  int margin = 40;
  bool quickTrigger = (amp0 > noiseFloor[0] + margin) ||
                      (amp1 > noiseFloor[1] + margin) ||
                      (amp2 > noiseFloor[2] + margin) ||
                      (amp3 > noiseFloor[3] + margin);

  if (!quickTrigger) {
    delay(1);
    return;
  }

  // ── 2. Capture full frame ──
  captureFrame(buf0, buf1, buf2, buf3);

  // ── 3. Preprocess ──
  removeMean(buf0, N);
  removeMean(buf1, N);
  removeMean(buf2, N);
  removeMean(buf3, N);

  // Check frame energy
  int rms0 = frameRMS(buf0, N);
  int rms1 = frameRMS(buf1, N);
  int rms2 = frameRMS(buf2, N);
  int rms3 = frameRMS(buf3, N);
  int maxRMS = max(max(rms0, rms1), max(rms2, rms3));

  int maxPeak = max(max(framePeak(buf0, N), framePeak(buf1, N)),
                    max(framePeak(buf2, N), framePeak(buf3, N)));

  if (maxRMS < TRIGGER_RMS || maxPeak < MIN_PEAK_AMP) {
    Serial.print("Below threshold: RMS="); Serial.print(maxRMS);
    Serial.print(" Peak="); Serial.println(maxPeak);
    return;
  }

  Serial.print("RMS: "); Serial.print(rms0);
  Serial.print(", "); Serial.print(rms1);
  Serial.print(", "); Serial.print(rms2);
  Serial.print(", "); Serial.print(rms3);
  Serial.print("  Peak: "); Serial.println(maxPeak);

  // Copy buffers for correlation (pre-emphasis is destructive)
  int16_t corr0[N], corr1[N], corr2[N], corr3[N];
  memcpy(corr0, buf0, sizeof(buf0));
  memcpy(corr1, buf1, sizeof(buf1));
  memcpy(corr2, buf2, sizeof(buf2));
  memcpy(corr3, buf3, sizeof(buf3));

  preEmphasis(corr0, N);
  preEmphasis(corr1, N);
  preEmphasis(corr2, N);
  preEmphasis(corr3, N);

  // ── 4. TDOA via cross-correlation ──
  // lagX: Left(3) vs Right(1) — positive = sound from left
  // lagY: Front(0) vs Back(2) — positive = sound from front
  int lagX = lagCorrelation(corr3, corr1, N, LAG_MAX);
  int lagY = lagCorrelation(corr0, corr2, N, LAG_MAX);

  float dtX_us = (1e6f * (float)lagX) / (float)FS_HZ;
  float dtY_us = (1e6f * (float)lagY) / (float)FS_HZ;

  Serial.print("lagX="); Serial.print(lagX);
  Serial.print(" ("); Serial.print(dtX_us); Serial.print("us)");
  Serial.print("  lagY="); Serial.print(lagY);
  Serial.print(" ("); Serial.print(dtY_us); Serial.println("us)");

  // ── 5. Motor intensity from frame energy ──
  int pwmMax = max(max(rmsToMotorPWM(rms0), rmsToMotorPWM(rms1)),
                   max(rmsToMotorPWM(rms2), rmsToMotorPWM(rms3)));

  // ── 6. Activate motor based on direction ──
  int motorFront = 0, motorRight = 0, motorBack = 0, motorLeft = 0;
  int absX = abs(lagX);
  int absY = abs(lagY);

  if (absX == 0 && absY == 0) {
    // No directional cue — gentle buzz all around
    Serial.println(">>> OMNI");
    motorFront = motorRight = motorBack = motorLeft = pwmMax / 4;
  }
  else if (absX > absY) {
    // Predominantly left or right
    if (lagX > 0) {
      Serial.println(">>> LEFT");
      motorLeft = pwmMax;
    } else {
      Serial.println(">>> RIGHT");
      motorRight = pwmMax;
    }
    // Blend secondary axis
    if (absY > 0) {
      int blend = pwmMax / 3;
      if (lagY > 0) motorFront = blend;
      else          motorBack  = blend;
    }
  }
  else if (absY > absX) {
    // Predominantly front or back
    if (lagY > 0) {
      Serial.println(">>> FRONT");
      motorFront = pwmMax;
    } else {
      Serial.println(">>> BACK");
      motorBack = pwmMax;
    }
    if (absX > 0) {
      int blend = pwmMax / 3;
      if (lagX > 0) motorLeft  = blend;
      else          motorRight = blend;
    }
  }
  else {
    // Equal diagonal — activate two motors
    int diag = pwmMax * 2 / 3;
    if      (lagX > 0 && lagY > 0) { Serial.println(">>> FRONT-LEFT");  motorFront = diag; motorLeft  = diag; }
    else if (lagX < 0 && lagY > 0) { Serial.println(">>> FRONT-RIGHT"); motorFront = diag; motorRight = diag; }
    else if (lagX > 0 && lagY < 0) { Serial.println(">>> BACK-LEFT");   motorBack  = diag; motorLeft  = diag; }
    else                           { Serial.println(">>> BACK-RIGHT");  motorBack  = diag; motorRight = diag; }
  }

  // Enforce motor minimum (below this they won't spin)
  if (motorFront > 0) motorFront = max(motorFront, MOTOR_MIN_PWM);
  if (motorRight > 0) motorRight = max(motorRight, MOTOR_MIN_PWM);
  if (motorBack  > 0) motorBack  = max(motorBack,  MOTOR_MIN_PWM);
  if (motorLeft  > 0) motorLeft  = max(motorLeft,   MOTOR_MIN_PWM);

  // Write PWM
  analogWrite(motorPin0, constrain(motorFront, 0, 255));
  analogWrite(motorPin1, constrain(motorRight, 0, 255));
  analogWrite(motorPin2, constrain(motorBack,  0, 255));
  analogWrite(motorPin3, constrain(motorLeft,  0, 255));

  delay(MOTOR_PULSE_MS);

  // Motors off
  analogWrite(motorPin0, 0);
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);

  // Cooldown to avoid retriggering on the same sound
  delay(20);
}
