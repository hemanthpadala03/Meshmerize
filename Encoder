#define MotFwd  10  // Motor Forward pin
#define MotRev  11 // Motor Reverse pin
#define MotFwd2 12  // Motor Forward pin
#define MotRev2  13 // Motor Reverse pin

int encoderPin1 = 2; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.
int encoderPin3 = 4; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin4 = 5;

volatile int MSB2= 0;
volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value

volatile int lastEncoded2 = 0; // Here updated value of encoder store.
volatile long encoderValue2 = 0; // Raw encoder value

void setup() {

  pinMode(MotFwd, OUTPUT); 
  pinMode(MotRev, OUTPUT); 
  Serial.begin(9600); //initialize serial comunication

   pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);
   pinMode(encoderPin3, INPUT_PULLUP); 
  pinMode(encoderPin4, INPUT_PULLUP);
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  pinMode(MotFwd2, OUTPUT); 
  pinMode(MotRev2, OUTPUT); 
  Serial.begin(9600); //initialize serial comunication
   pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);
   pinMode(encoderPin3, INPUT_PULLUP); 
  pinMode(encoderPin4, INPUT_PULLUP);

  digitalWrite(encoderPin3, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin4, HIGH); //turn pullup resistor on


  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);
  


}

void loop() {

for (int i = 0; i <= 200; i++){
digitalWrite(MotFwd, LOW); 
 digitalWrite(MotRev, HIGH);
 Serial.print("Forward  ");
 Serial.print(encoderValue);
 digitalWrite(MotFwd2, LOW); 
 digitalWrite(MotRev2, HIGH);
 Serial.print("Forward  ");
 Serial.println(encoderValue2);
}


for (int i = 0; i <= 200; i++){
digitalWrite(MotFwd, HIGH); 
 digitalWrite(MotRev, LOW);
 Serial.print("Reverse  ");
 Serial.print(encoderValue);
 digitalWrite(MotFwd2, HIGH); 
 digitalWrite(MotRev2, LOW);
 Serial.print("Reverse  ");
 Serial.println(encoderValue2);
// Serial.println(MSB);
}


} 

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue ++;

  lastEncoded = encoded; //store this value for next time

}
void updateEncoder2(){
  int MSB2 = digitalRead(encoderPin3); //MSB = most significant bit
  int LSB2 = digitalRead(encoderPin4); //LSB = least significant bit

  int encoded2 = (MSB2 << 1) |LSB2; //converting the 2 pin value to single number
  int sum2  = (lastEncoded2 << 2) | encoded2; //adding it to the previous encoded value

  if(sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) encoderValue2 --;
  if(sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) encoderValue2 ++;

  lastEncoded2 = encoded2; //store this value for next time
}
