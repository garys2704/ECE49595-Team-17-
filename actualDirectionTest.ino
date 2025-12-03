const int micPinR = A2;      
const int motorPinR = 10;    
const int micPinL = A3;
const int motorPinL = 9;
const int micCenterR = 310;
const int micCenterL = 300;
const int triggerThresholdR = 60;
const int triggerThresholdL = 60;

void setup() {
  Serial.begin(9600);
  pinMode(motorPinL, OUTPUT);
  pinMode(motorPinR, OUTPUT);
  analogWrite(motorPinR, 0); 
  analogWrite(motorPinL, 0);
}

void loop() {

  int sensorValueR = analogRead(micPinR);
  int sensorValueL = analogRead(micPinL);
  int amplitudeR = abs(sensorValueR - micCenterR);
  int amplitudeL = abs(sensorValueL - micCenterL);
  int minLoudnessR = 1;   
  int maxLoudnessR = 420;
  int minLoudnessL = 1;   
  int maxLoudnessL = 420;
  int vibrationIntensityR = map(amplitudeR, minLoudnessR, maxLoudnessR, 0, 255);
  int vibrationIntensityL = map(amplitudeL, minLoudnessL, maxLoudnessL, 0, 255);

  vibrationIntensityR = constrain(vibrationIntensityR, 0, 255);
  vibrationIntensityL = constrain(vibrationIntensityL, 0, 255);
  int motorPwmValueR = vibrationIntensityR;
  int motorPwmValueL = vibrationIntensityL; 

  //analogWrite(motorPin, motorPwmValue); 

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

  if (amplitudeR > triggerThresholdR || amplitudeL > triggerThresholdL)
  {
    if (amplitudeR > amplitudeL)
    {
      Serial.println("Right detected first");
      analogWrite(motorPinR, motorPwmValueR);
    }
    else if (amplitudeL > amplitudeR)
    {
      Serial.println("Left detected first");
      analogWrite(motorPinL, motorPwmValueL);
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
