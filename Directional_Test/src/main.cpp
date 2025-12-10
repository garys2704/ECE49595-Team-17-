#include <Arduino.h>

// PIN DEFINITIONS
// On the Nucleo F767ZI, the "Arduino" headers match the standard layout.
// A2 and A3 are on the CN9 connector.
// D9 and D10 are on the CN5 connector.
const int micPinR = A2;      
const int motorPinR = 10;    
const int micPinL = A3;
const int motorPinL = 9;

// TUNING VARIABLES
// NOTE: STM32 is 3.3V. If your mic sits at 1.65V (half VCC) when quiet,
// on 10-bit resolution, this value should be around 512. 
// Your old value was 310/300. CHECK SERIAL MONITOR TO RETUNE THIS!
const int micCenterR = 512; 
const int micCenterL = 512; 

const int triggerThresholdR = 60; 
const int triggerThresholdL = 60; 

void setup() {
  // STM32 is much faster, so we use a faster baud rate
  Serial.begin(115200);

  // CRITICAL: Standard Arduino is 10-bit (0-1023). 
  // STM32 defaults to 12-bit (0-4095). 
  // We set this to 10 bits so your logic and thresholds (like 60) remain similar.
  analogReadResolution(10); 

  pinMode(motorPinL, OUTPUT);
  pinMode(motorPinR, OUTPUT);
  
  // Initialize motors to off
  analogWrite(motorPinR, 0); 
  analogWrite(motorPinL, 0);
  
  Serial.println("System Initialized - STM32F767ZI");
}

void loop() {
  // 1. Read Sensors
  int sensorValueR = analogRead(micPinR); 
  int sensorValueL = analogRead(micPinL);

  // 2. Calculate Amplitude (Distance from "Quiet" voltage)
  int amplitudeR = abs(sensorValueR - micCenterR); 
  int amplitudeL = abs(sensorValueL - micCenterL);

  // 3. Define Loudness Range
  int minLoudnessR = 1;  
  int maxLoudnessR = 420;
  int minLoudnessL = 1;   
  int maxLoudnessL = 420;

  // 4. Map Amplitude to PWM (0-255)
  // Explanation: This scales the "amplitude" (input) from a range of 1-420
  // to a range of 0-255 (output) for the motor speed.
  int vibrationIntensityR = map(amplitudeR, minLoudnessR, maxLoudnessR, 0, 255); 
  int vibrationIntensityL = map(amplitudeL, minLoudnessL, maxLoudnessL, 0, 255);

  // 5. Constrain (Safety Clipping)
  // Ensure we don't send negative numbers or numbers > 255 to the motor
  vibrationIntensityR = constrain(vibrationIntensityR, 0, 255);
  vibrationIntensityL = constrain(vibrationIntensityL, 0, 255);

  int motorPwmValueR = vibrationIntensityR;
  int motorPwmValueL = vibrationIntensityL; 

  // 6. Debugging
  // Use a formatted string for cleaner STM32 debugging output? Optional but nice.
  // We will stick to your format for familiarity.
  Serial.print("RawR: "); Serial.print(sensorValueR);
  Serial.print(" | RawL: "); Serial.print(sensorValueL);
  Serial.print(" | AmpR: "); Serial.print(amplitudeR);
  Serial.print(" | AmpL: "); Serial.print(amplitudeL);
  Serial.print(" | MotorR: "); Serial.print(motorPwmValueR);
  Serial.print(" | MotorL: "); Serial.println(motorPwmValueL);

  // 7. Detection Logic
  // Check if either mic crossed the threshold
  if (amplitudeR > triggerThresholdR || amplitudeL > triggerThresholdL) 
  {
    if (amplitudeR > amplitudeL) // Right is louder
    {
      Serial.println(">>> Right Source Detected");
      analogWrite(motorPinR, motorPwmValueR);
      // Ensure the other motor is off or low? Your original logic didn't turn off Left here, 
      // but usually, you want to clear the other channel:
      analogWrite(motorPinL, 0); 
    }
    else if (amplitudeL > amplitudeR) // Left is louder
    {
      Serial.println("<<< Left Source Detected");
      analogWrite(motorPinL, motorPwmValueL);
      analogWrite(motorPinR, 0);
    }
    else // Exactly Equal
    {
      Serial.println("=== Center Source Detected");
      analogWrite(motorPinR, motorPwmValueR / 2);
      analogWrite(motorPinL, motorPwmValueL / 2);
    }
  }
  else
  {
    // Below threshold - Turn everything off
    analogWrite(motorPinR, 0);
    analogWrite(motorPinL, 0);
  }

  delay(5); 
}