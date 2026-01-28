#include <Arduino.h>

// PIN DEFINITIONS
// On the Nucleo F767ZI, the "Arduino" headers match the standard layout.
// A2 and A3 are on the CN9 connector.
// D9 and D10 are on the CN5 connector.
const int micPin1 = A1;      
const int motorPin1 = 10;    
const int micPin2 = A2;
const int motorPin2 = 9;
const int micPin3 = A3;      
const int motorPin3 = 8;    
const int micPin4 = A4;
const int motorPin4 = 7;

// TUNING VARIABLES
// NOTE: STM32 is 3.3V. If your mic sits at 1.65V (half VCC) when quiet,
// on 10-bit resolution, this value should be around 512. 
// Your old value was 310/300. CHECK SERIAL MONITOR TO RETUNE THIS!
const int micCenter1 = 512; 
const int micCenter2 = 512; 
const int micCenter3 = 512; 
const int micCenter4 = 512; 

const int triggerThreshold1 = 60; 
const int triggerThreshold2 = 60;
const int triggerThreshold3 = 60; 
const int triggerThreshold4 = 60; 

void setup() {
  
  Serial.begin(115200);

  // CRITICAL: Standard Arduino is 10-bit (0-1023). 
  // STM32 defaults to 12-bit (0-4095). 
  // We set this to 10 bits so your logic and thresholds (like 60) remain similar.
  analogReadResolution(10); 

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  
  analogWrite(motorPin1, 0); 
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0); 
  analogWrite(motorPin4, 0);
  
  Serial.println("System Initialized - STM32F767ZI");
}

void loop() {
  // 1. Read Sensors
  int sensorValue1 = analogRead(micPin1); 
  int sensorValue2 = analogRead(micPin2);
  int sensorValue3 = analogRead(micPin3); 
  int sensorValue4 = analogRead(micPin4);

  // 2. Calculate Amplitude (Distance from "Quiet" voltage)
  int amplitude1 = abs(sensorValue1 - micCenter1); 
  int amplitude2 = abs(sensorValue2 - micCenter2);
  int amplitude3 = abs(sensorValue3 - micCenter3); 
  int amplitude4 = abs(sensorValue4 - micCenter4);
  // 3. Define Loudness Range
  int minLoudness1 = 1;
  int minLoudness2 = 1;
  int minLoudness3 = 1;
  int minLoudness4 = 1;

  int maxLoudness1 = 420;
  int maxLoudness2 = 420;
  int maxLoudness3 = 420;
  int maxLoudness4 = 420;

  // 4. Map Amplitude to PWM (0-255)
  // Explanation: This scales the "amplitude" (input) from a range of 1-420
  // to a range of 0-255 (output) for the motor speed.
  int vibrationIntensity1 = map(amplitude1, minLoudness1, maxLoudness1, 0, 255); 
  int vibrationIntensity2 = map(amplitude2, minLoudness2, maxLoudness2, 0, 255);
  int vibrationIntensity3 = map(amplitude3, minLoudness3, maxLoudness3, 0, 255); 
  int vibrationIntensity4 = map(amplitude4, minLoudness4, maxLoudness4, 0, 255);
  // 5. Constrain (Safety Clipping)
  // Ensure we don't send negative numbers or numbers > 255 to the motor
  vibrationIntensity1 = constrain(vibrationIntensity1, 0, 255);
  vibrationIntensity2 = constrain(vibrationIntensity2, 0, 255);
  vibrationIntensity3 = constrain(vibrationIntensity3, 0, 255);
  vibrationIntensity4 = constrain(vibrationIntensity4, 0, 255);

  int motorPwmValue1 = vibrationIntensity1;
  int motorPwmValue2 = vibrationIntensity2; 
  int motorPwmValue3 = vibrationIntensity3;
  int motorPwmValue4 = vibrationIntensity4;

  // 6. Debugging
  // Use a formatted string for cleaner STM32 debugging output? Optional but nice.
  // We will stick to your format for familiarity.
  Serial.print("Raw1: "); Serial.print(sensorValue1);
  Serial.print(" | Raw2: "); Serial.print(sensorValue2);
  Serial.print(" | Raw3: "); Serial.print(sensorValue3);
  Serial.print(" | Raw4: "); Serial.print(sensorValue4);
  Serial.print(" | Amp1: "); Serial.print(amplitude1);
  Serial.print(" | Amp2: "); Serial.print(amplitude2);
  Serial.print(" | Amp3: "); Serial.print(amplitude1);
  Serial.print(" | Amp4: "); Serial.print(amplitude2);
  /*Serial.print(" | Motor1: "); Serial.print(motorPwmValue1);
  Serial.print(" | Motor2: "); Serial.println(motorPwmValue2);*/ 

  // 7. Detection Logic
  // Check if either mic crossed the threshold
  if (amplitude1 > triggerThreshold1 || amplitude2 > triggerThreshold2 || amplitude3 > triggerThreshold3 || amplitude4 > triggerThreshold4) 
  {
    if (amplitude1 > amplitude1) // mic 1 is louder
    {
      Serial.println(">>> Source 1 Detected");
      analogWrite(motorPin1, motorPwmValue1);
      // Ensure the other motor is off or low? Your original logic didn't turn off Left here, 
      // but usually, you want to clear the other channel:
      analogWrite(motorPin2, 0);
      analogWrite(motorPin3, 0);
      analogWrite(motorPin4, 0); 
    }
    else if (amplitude2 > amplitude2) // mic 2 is louder
    {
      Serial.println("<<< Source 2 Detected");
      analogWrite(motorPin2, motorPwmValue2);
      analogWrite(motorPin1, 0);
      analogWrite(motorPin3, 0);
      analogWrite(motorPin4, 0);
    }
    else if (amplitude3 > amplitude3) // mic 3 is louder
    {
      Serial.println("<<< Source 3 Detected");
      analogWrite(motorPin3, motorPwmValue3);
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, 0);
      analogWrite(motorPin4, 0);
    }
    else if (amplitude4 > amplitude4) // mic 4 is louder
    {
      Serial.println("<<< Source 4 Detected");
      analogWrite(motorPin4, motorPwmValue4);
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, 0);
      analogWrite(motorPin3, 0);
    }
    /*else // NEED TO FIX (comment out for directional testing)
    {
      Serial.println("=== Center Source Detected");
      analogWrite(motorPin1, motorPwmValue1 / 2);
      analogWrite(motorPin2, motorPwmValue2 / 2);
    }*/
  }
  else
  {
    // Below threshold - Turn everything off
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 0);
  }

  delay(5); 
}
