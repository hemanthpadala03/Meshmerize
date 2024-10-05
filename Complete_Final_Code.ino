#include <QTRSensors.h>

QTRSensors qtr;

#define Kp  0.085
// experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Ki 0.0
#define Kd  0.5
#define rightMaxSpeed 220 // max speed of the robot                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
#define leftMaxSpeed 220// max speed of the robot
#define rightBaseSpeed 170 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 170
const int encoderPin1 = 18;
const int encoderPin2 = 19;
const int encoderPin3 = 20;
const int encoderPin4 = 21;
volatile int lastEncoded1 = 0;
volatile long encoderValue1 = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;
char directionArray[100]; // Assuming a max size of 100
int enco[100];
int index = 0;
int lastError = 0;
int I;
int pos=0;
const uint8_t SensorCount =10;
uint16_t sensorValues[SensorCount];
volatile int x=0;
int b=800;
int t=1;
int yt;
int k=1;


#define rightenable 13 //enableB
#define leftenable 7 //enableA
#define rightMotor1 11 //int3
#define rightMotor2 12 //int4
#define leftMotor1 9 //int1
#define leftMotor2 10 //int2

void setup()
{
   pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightenable, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftenable, OUTPUT);
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP);
  pinMode(encoderPin4, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4), updateEncoder2, CHANGE);


  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5,A6,A7,A8,A9}, SensorCount);
  //qtr.setEmitterPin(2); 

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 100; i++)
  {
      digitalWrite(rightenable, HIGH);
      digitalWrite(leftenable, HIGH);// move forward with appropriate speeds
      analogWrite(rightMotor1, 0);
      analogWrite(rightMotor2, 150);  
      analogWrite(leftMotor1, 150);
      analogWrite(leftMotor2, 0);
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

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
  uint16_t position = qtr.readLineBlack(sensorValues);

  int pos = -(4*sensorValues[1]+3*sensorValues[2]+2*sensorValues[3]+sensorValues[4]-sensorValues[5]-2*sensorValues[6]-3*sensorValues[7]-4*sensorValues[8]);
//  Serial.println(pos);
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
    if (rightMotorSpeed < -rightMaxSpeed){
      rightMotorSpeed = -rightMaxSpeed; // keep the motor speed positive
    }
    if (leftMotorSpeed < -leftMaxSpeed){
      leftMotorSpeed = -leftMaxSpeed; // keep the motor speed positive
    }
    digitalWrite(rightenable, HIGH);
    digitalWrite(leftenable, HIGH);// move forward with appropriate speeds
    if(sensorValues[0]>700 && sensorValues[1]>700 && sensorValues[2]>700 && sensorValues[3]>700 && sensorValues[4]>700 && sensorValues[5]>700 && sensorValues[6]>700 && sensorValues[7]>700 && sensorValues[8]>700 && sensorValues[9]>700 ){
       if(k>0){
        directionArray[index]='B';
      enco[index]=encoderValue1;
      yt=encoderValue1;
      index=index+1;
        k=0;
      }
      if(encoderValue1>yt+300){
        k=1;
      }   
       analogWrite(rightMotor1, 250);
       analogWrite(rightMotor2, 0);
       analogWrite(leftMotor1, 0);
       analogWrite(leftMotor2, 100);
      
   
    }else if( sensorValues[3]<200 && sensorValues[4]<200 && sensorValues[5]<200 && sensorValues[6]<200 && sensorValues[7]<200 && sensorValues[8]<200 && sensorValues[9]<200 ){
//       Serial.println("Intersection");
  
      analogWrite(rightMotor1, 0);
      analogWrite(rightMotor2, 100);
      analogWrite(leftMotor1, 250);
      analogWrite(leftMotor2, 0);
      if(t>0){
        directionArray[index]='L';
        enco[index]=encoderValue1;
        yt=encoderValue1;
        index=index+1;
        t=0;
      }
      if(encoderValue1>yt+300){
        t=1;
      }
      
    }else{
      if (leftMotorSpeed>0){
        analogWrite(rightMotor1, 0);
      analogWrite(rightMotor2, leftMotorSpeed);
      }else{
        analogWrite(rightMotor1, -leftMotorSpeed);
      analogWrite(rightMotor2, 0);
      }
      if (rightMotorSpeed>0){
        analogWrite(leftMotor1, 0);
      analogWrite(leftMotor2, rightMotorSpeed);
      }else{
        analogWrite(leftMotor1, -rightMotorSpeed);
      analogWrite(leftMotor2, 0);
      }
    }
   shortPath()
}
void shortPath()
{
	int shortDone = 0;
	if(path[pathLength-3] == 'L' && path[pathLength - 1] == 'R')
	{
		pathLength -= 3;
		path[pathLength] = 'B';
		shortDone = 1;
	}
	if(path[pathLength-3] == 'L' && path[pathLength - 1] == 'S' && shortDone == 0)
	{
		pathLength -= 3;
		path[pathLength] = 'R';
		shortDone = 1;
	}
	if(path[pathLength-3] == 'R' && path[pathLength - 1] == 'L' && shortDone == 0)
	{
		pathLength-=3;
		path[pathLength] = 'B';
		shortDone=1;
	}
	if(path[pathLength-3] == 'S' && path[pathLength - 1] == 'L' && shortDone == 0)
	{
		pathLength -= 3;
		path[pathLength] = 'R';
		shortDone = 1;
	}
	if(path[pathLength-3] == 'S' && path[pathLength - 1] =='S' && shortDone == 0)
	{
		pathLength-=3;
		path[pathLength] = 'B';
		shortDone=1;
	}
	if(path[pathLength-3] == 'L' && path[pathLength - 1] =='L' && shortDone == 0)
	{
		pathLength -= 3;
		path[pathLength] = 'S';
		shortDone = 1;
	}
	path[pathLength+1] = 'D';
	path[pathLength+2] = 'D';
	pathLength++;
}
void updateEncoder() {
  int MSB1 = digitalRead(encoderPin1);
  int LSB1 = digitalRead(encoderPin2);
  
  int encoded1 = (MSB1 << 1) | LSB1;
  int sum1 = (lastEncoded1 << 2) | encoded1;

  if (sum1 == 0b1101 || sum1 == 0b0100 || sum1 == 0b0010 || sum1 == 0b1011)
    encoderValue1--;
  if (sum1 == 0b1110 || sum1 == 0b0111 || sum1 == 0b0001 || sum1 == 0b1000)
    encoderValue1++;

  lastEncoded1 = encoded1;
}

void updateEncoder2() {
  int MSB2 = digitalRead(encoderPin3);
  int LSB2 = digitalRead(encoderPin4);
  int encoded2 = (MSB2 << 1) | LSB2;
  int sum2 = (lastEncoded2 << 2) | encoded2;

  if (sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011)
    encoderValue2--;
  if (sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000)
    encoderValue2++;

  lastEncoded2 = encoded2;
}
