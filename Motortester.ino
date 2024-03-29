#include <Arduino.h> // Include the Arduino library to use Serial

#define rightenable 13 //enableB
#define leftenable 8 //enableA
#define rightMotor1 12 //int3
#define rightMotor2 11 //int4
#define leftMotor1 9 //int1
#define leftMotor2 10 //int2

void setup() {
 pinMode(rightMotor1, OUTPUT);
 pinMode(rightMotor2, OUTPUT);
 pinMode(rightenable, OUTPUT);
 pinMode(leftMotor1, OUTPUT);
 pinMode(leftMotor2, OUTPUT);
 pinMode(leftenable, OUTPUT);

 Serial.begin(9600); // Start the serial communication
}

void loop() {
 if (Serial.available() > 0) { // Check if data is available to read
    String dataString = Serial.readStringUntil('\n'); // Read the incoming data until newline
    int data = dataString.toInt(); // Convert the string to an integer

    Serial.print("Received: ");
    Serial.println(data); // Print the received integer value

    digitalWrite(rightenable, HIGH);
    digitalWrite(leftenable, HIGH); // Enable motors

    analogWrite(rightMotor1, 0);
    analogWrite(rightMotor2, data); // Use the integer value for analogWrite
    analogWrite(leftMotor1, data);
    analogWrite(leftMotor2, 0);

    delay(4500); // Wait for 4.5 seconds
 }
}
