const int micPinR = A2;      
const int motorPinR = 10;    
const int micPinL = A3;
const int motorPinL = 9;
const int micCenterR = 310; // need to be tuned based on environment
const int micCenterL = 300; 
const int triggerThresholdR = 60; // these are the values that control the sensitivity of the microphones (trigger threshold left and right)
const int triggerThresholdL = 60; 

void setup() {
  Serial.begin(9600);
  pinMode(motorPinL, OUTPUT);
  pinMode(motorPinR, OUTPUT);
  analogWrite(motorPinR, 0); 
  analogWrite(motorPinL, 0);
}

void loop() {

  int sensorValueR = analogRead(micPinR); //read in microphone sensor level
  int sensorValueL = analogRead(micPinL);
  int amplitudeR = abs(sensorValueR - micCenterR); // calculate the amplitude of the signal by subtracting the sensor reading from the "quiet level of the microphone
  int amplitudeL = abs(sensorValueL - micCenterL);
  int minLoudnessR = 1;  // need to be tuned based on the environment 
  int maxLoudnessR = 420;
  int minLoudnessL = 1;   
  int maxLoudnessL = 420;
  int vibrationIntensityR = map(amplitudeR, minLoudnessR, maxLoudnessR, 0, 255); //ngl i dont really understand this part, thx gemini
  int vibrationIntensityL = map(amplitudeL, minLoudnessL, maxLoudnessL, 0, 255);

  vibrationIntensityR = constrain(vibrationIntensityR, 0, 255);
  vibrationIntensityL = constrain(vibrationIntensityL, 0, 255);
  int motorPwmValueR = vibrationIntensityR;
  int motorPwmValueL = vibrationIntensityL; 

// debugging and sensor reading
  Serial.print("Sensor Value right: ");
  Serial.print(sensorValueR);
  Serial.print(" | Sensor Value left: ");
  Serial.print(sensorValueL);
  Serial.print(" | Amplitude right: ");
  Serial.print(amplitudeR);
  Serial.print(" | Amplitude left: ");
  Serial.print(amplitudeL);
  Serial.print(" | PWM Value (Right Motor): ");
  Serial.print(motorPwmValueR);
  Serial.print(" | PWM Value (Left Motor): ");
  Serial.println(motorPwmValueL);

  if (amplitudeR > triggerThresholdR || amplitudeL > triggerThresholdL) //check if the right and left sensor readings are above the thresholds (from the constant variables, this is the sensitivity adjustment)
  {
    if (amplitudeR > amplitudeL) //checks if right is greater than left
    {
      Serial.println("Right detected first");
      analogWrite(motorPinR, motorPwmValueR);
    }
    else if (amplitudeL > amplitudeR) //checks if left if greater than right
    {
      Serial.println("Left detected first");
      analogWrite(motorPinL, motorPwmValueL); //sends out calculated pwm value to motor pin
    }
    else if (amplitudeR == amplitudeL)
    {
      Serial.println("Both detected at same time");
      int newPwmR = motorPwmValueR / 2;
      int newPwmL = motorPwmValueL / 2;
      analogWrite(motorPinR, newPwmR);
      analogWrite(motorPinL, newPwmL);
    }
  }
  else
  {
    analogWrite(motorPinR, 0);
    analogWrite(motorPinL, 0);
  }
  delay(5); 
}
