#define MotFwd1  9 // Motor 1 Forward pin
#define MotRev1  10 // Motor 1 Reverse pin
#define MotFwd2  11 // Motor 2 Forward pin
#define MotRev2  12 // Motor 2 Reverse pin
#define Enable1  8
#define Enable2  13

// Define constants for encoder pin numbers
const int encoderPin1 = 20;
const int encoderPin2 = 21;
const int encoderPin3 = 2;
const int encoderPin4 = 3;

// Define volatile variables for encoder state and value
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;

// Function prototypes
void updateEncoder();
void updateEncoder2();

void setup() {
  pinMode(Enable1, OUTPUT); 
 pinMode(Enable2, OUTPUT); 
 pinMode(MotFwd1, OUTPUT); 
 pinMode(MotRev1, OUTPUT); 
 pinMode(MotFwd2, OUTPUT); 
 pinMode(MotRev2, OUTPUT);
  Serial.begin(9600);

  // Set encoder pins as inputs and enable pull-up resistors
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  pinMode(encoderPin3, INPUT_PULLUP);
  pinMode(encoderPin4, INPUT_PULLUP);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin3), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin4), updateEncoder2, CHANGE);
}

void loop() {
    digitalWrite(Enable1, HIGH); 
    digitalWrite(Enable2, HIGH); 
   digitalWrite(MotFwd1, HIGH); 
   digitalWrite(MotRev1, LOW);
   digitalWrite(MotFwd2, HIGH); 
   digitalWrite(MotRev2, LOW);
  // Print encoder values
  Serial.print("Value for left motor ");
  Serial.println(encoderValue);
  Serial.print("Value for right motor  ");
  Serial.println(encoderValue2);
  delay(1000); // Delay to slow down output
}

void updateEncoder() {
  int MSB = digitalRead(encoderPin1);
  int LSB = digitalRead(encoderPin2);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // Update encoder value based on direction of rotation
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValue--;

  // Update lastEncoded value
  lastEncoded = encoded;
}

void updateEncoder2() {
  int MSB2 = digitalRead(encoderPin3);
  int LSB2 = digitalRead(encoderPin4);
  int encoded2 = (MSB2 << 1) | LSB2;
  int sum2 = (lastEncoded2 << 2) | encoded2;

  // Update encoder value based on direction of rotation
  if (sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011)
    encoderValue2++;
  if (sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000)
    encoderValue2--;

  // Update lastEncoded2 value
  lastEncoded2 = encoded2;
}
