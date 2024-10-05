#include <QTRSensors.h>

QTRSensors qtr;

#define Kp  0.059
// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Ki 0.0
#define Kd  0.0
#define rightMaxSpeed 200 // max speed of the robot                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
#define leftMaxSpeed 200// max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150
int lastError = 0;
int I;
int pos=0;
const uint8_t SensorCount =10;
uint16_t sensorValues[SensorCount];


#define rightenable 13 //enableB
#define leftenable 8 //enableA
#define rightMotor1 12 //int3
#define rightMotor2 11 //int4
#define leftMotor1 10 //int1
#define leftMotor2 9 //int2

void setup()
{
   pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightenable, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftenable, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(52, OUTPUT);
  

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6,A7,A8,A9}, SensorCount);
  qtr.setEmitterPin(2); 

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(50, HIGH);
  digitalWrite(52, LOW);
  for (uint16_t i = 0; i < 200; i++)
  {
      digitalWrite(rightenable, HIGH);
      digitalWrite(leftenable, HIGH);// move forward with appropriate speeds
      analogWrite(rightMotor1, 0);
      analogWrite(rightMotor2, 150);
      analogWrite(leftMotor1, 0);
      analogWrite(leftMotor2, 0);
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  digitalWrite(50, LOW);
  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    
  }
  Serial.print('\n');
  int pos = -(3*sensorValues[0]+2.5*sensorValues[1]+2*sensorValues[2]+1.5*sensorValues[3]+sensorValues[4]-sensorValues[5]-1.5*sensorValues[6]-2*sensorValues[7]-2.5*sensorValues[8]-3*sensorValues[9]);
  Serial.println(pos);
  int error = pos + 22;
    I = I + error ;
    int motorSpeed = Kp * error + Kd * (error - lastError) + Ki * (I);
    lastError = error;

    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;

    if (rightMotorSpeed > rightMaxSpeed ){
        rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
    }
    if (leftMotorSpeed > leftMaxSpeed ){
      leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
    }
    if (rightMotorSpeed < 0){
      rightMotorSpeed = 0; // keep the motor speed positive
    }
    if (leftMotorSpeed < 0){
      leftMotorSpeed = 0; // keep the motor speed positive
    }
      digitalWrite(rightenable, HIGH);
      digitalWrite(leftenable, HIGH);// move forward with appropriate speeds
      analogWrite(rightMotor1, 0);
      analogWrite(rightMotor2, leftMotorSpeed);
      analogWrite(leftMotor1, 0);
      analogWrite(leftMotor2, rightMotorSpeed);
    Serial.println(rightMotorSpeed);
    Serial.println(leftMotorSpeed);

  delay(250);
}
