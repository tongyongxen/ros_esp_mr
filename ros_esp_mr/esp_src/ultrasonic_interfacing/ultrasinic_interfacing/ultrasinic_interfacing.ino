
#include <ESP32Encoder.h>

#define IN3 26 // Left wheel
#define IN4 27
#define PWM_L_PIN 32
#define IN1 12 //Right Wheel
#define IN2 14
#define PWM_R_PIN 15 

#define C1_L 25 //Left encoder
#define C2_L 33 
#define C1_R 4 //Right encoder
#define C2_R 2
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;
volatile long encoderCountLeft = 0;
volatile long encoderCountRight = 0;

void setup() {
  Serial.begin(115200);

  // Set left motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_L_PIN, OUTPUT);

  // Set right motor control pins as outputs
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_R_PIN, OUTPUT);

  // Set left encoder pins as inputs
  pinMode(C1_L, INPUT_PULLUP);
  pinMode(C2_L, INPUT_PULLUP);

  // Set right encoder pins as inputs
  pinMode(C1_R, INPUT_PULLUP);
  pinMode(C2_R, INPUT_PULLUP);

  // Attach left encoder pins to the encoderLeft object
  encoderLeft.attachHalfQuad(C1_L, C2_L);

  // Attach right encoder pins to the encoderRight object
  encoderRight.attachHalfQuad(C1_R, C2_R);

  // Set PWM frequency for left motor control
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM_L_PIN, 0);

  // Set PWM frequency for right motor control
  ledcSetup(1, 5000, 8);
  ledcAttachPin(PWM_R_PIN, 1);
    stop();

}

void loop() {
  // Your main code goes here
  // You can use encoderCountLeft and encoderCountRight variables for the counts
  // Example: Print counts to Serial
  Serial.print("Left Encoder Count: ");
  Serial.print(encoderCountLeft);
  Serial.print(" | Right Encoder Count: ");
  Serial.println(encoderCountRight);

  delay(1000);  // You can adjust the delay based on your application
}

void leftEncoderISR() {
  if (digitalRead(C2_L) == HIGH) {
    encoderCountLeft++;
  } else {
    encoderCountLeft--;
  }
}

void rightEncoderISR() {
  if (digitalRead(C2_R) == HIGH) {
    encoderCountRight++;
  } else {
    encoderCountRight--;
  }
}

void stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Stop the right motor by setting both IN3 and IN4 to LOW
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Set PWM to 0 for both motors
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}
