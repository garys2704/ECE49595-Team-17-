#include <Arduino.h>

const int micPinR = A2;      
const int motorPinR = 10;    
const int micPinL = A3;
const int motorPinL = 9;

const int micCenterR = 512; 
const int micCenterL = 512; 

const int triggerThresholdR = 60; 
const int triggerThresholdL = 60; 

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
  int minLoudnessR = 1;  
  int maxLoudnessR = 420;
  int minLoudnessL = 1;   
  int maxLoudnessL = 420;

  // map Amplitude to PWM (0-255) (scales amplitude to motor speed)
  int vibrationIntensityR = map(amplitudeR, minLoudnessR, maxLoudnessR, 0, 255); 
  int vibrationIntensityL = map(amplitudeL, minLoudnessL, maxLoudnessL, 0, 255);

  // constrain (Safety Clipping) (no negative values to motors)
  vibrationIntensityR = constrain(vibrationIntensityR, 0, 255);
  vibrationIntensityL = constrain(vibrationIntensityL, 0, 255);

  int motorPwmValueR = vibrationIntensityR;
  int motorPwmValueL = vibrationIntensityL; 

  // Serial monitor formatting for prototyping
  Serial.print("RawR: "); Serial.print(sensorValueR);
  Serial.print(" | RawL: "); Serial.print(sensorValueL);
  Serial.print(" | AmpR: "); Serial.print(amplitudeR);
  Serial.print(" | AmpL: "); Serial.print(amplitudeL);
  Serial.print(" | MotorR: "); Serial.print(motorPwmValueR);
  Serial.print(" | MotorL: "); Serial.println(motorPwmValueL);

  // detection Logic to check if mics crossed the threshold
  if (amplitudeR > triggerThresholdR || amplitudeL > triggerThresholdL) 
  {
    if (amplitudeR > amplitudeL) // Right is louder
    {
      Serial.println(">>> Right Source Detected");
      analogWrite(motorPinR, motorPwmValueR); // send pwm value to detected motor
      analogWrite(motorPinL, 0); // make sure the other one is off
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
      analogWrite(motorPinR, motorPwmValueR / 2); // send half the pwm value to each if the detected amplitudes are the same (sound detected in the middle of the two)
      analogWrite(motorPinL, motorPwmValueL / 2);
    }
  }
  else
  {
    // below threshold - turn everything off
    analogWrite(motorPinR, 0);
    analogWrite(motorPinL, 0);
  }

  delay(5); 

}
