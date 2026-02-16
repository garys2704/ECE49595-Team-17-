#include <Arduino.h>

const int micPinR = A2;      
const int motorPinR = 6;    
const int micPinL = A3;
const int motorPinL = 5;

const int micCenterR = 99; 
const int micCenterL = 5; 

const int triggerThresholdR = 100; 
const int triggerThresholdL = 2; 

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

  int motorPwmValueR = vibrationIntensityR;
  int motorPwmValueL = vibrationIntensityL; 

  // Serial monitor formatting for prototyping
  //Serial.print("RawR: "); Serial.print(sensorValueR);
  Serial.print(" | RawL: "); Serial.print(sensorValueL);
  //Serial.print(" | AmpR: "); Serial.print(amplitudeR);
  Serial.print(" | AmpL: "); Serial.print(amplitudeL);
  //Serial.print(" | MotorR: "); Serial.print(motorPwmValueR);
  Serial.print(" | MotorL: "); Serial.println(motorPwmValueL);
  //Serial.print(" | vibrationIntensityR: "); Serial.println(vibrationIntensityR);

  // detection Logic to check if mics crossed the threshold
  if (amplitudeL > triggerThresholdL) 
  {
      Serial.println(">>> Left Source Detected");
      analogWrite(motorPinL, motorPwmValueL); // send pwm value to detected motor
      analogWrite(motorPinL, 0); // make sure the other one is off
  
    /*else if (amplitudeL > amplitudeR) // Left is louder
    {
      Serial.println("<<< Left Source Detected");
      analogWrite(motorPinL, motorPwmValueL);
      analogWrite(motorPinR, 0);
    }*/
    /*else // Exactly Equal
    {
      Serial.println("=== Center Source Detected");
      analogWrite(motorPinR, motorPwmValueR / 2); // send half the pwm value to each if the detected amplitudes are the same (sound detected in the middle of the two)
      analogWrite(motorPinL, motorPwmValueL / 2);
    }*/
  }
  else
  {
    // below threshold - turn everything off
    analogWrite(motorPinR, 0);
    analogWrite(motorPinL, 0);
  }

  delay(5); 

}
