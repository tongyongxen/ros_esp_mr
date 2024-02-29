#include <ESP32Encoder.h>

// Define motor control pins
#define IN3 26
#define IN4 27
#define PWM_PIN 32

// Define encoder pins
#define C1 25
#define C2 33

// Create an encoder object
ESP32Encoder encoder;

void setup() {
    Serial.begin(115200);
  // Set motor control pins as outputs
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  // Set encoder pins as inputs
  pinMode(C1, INPUT_PULLUP);
  pinMode(C2, INPUT_PULLUP);

  // Attach encoder pins to the encoder object
  encoder.attachHalfQuad(C1, C2);

  // Set PWM frequency for motor control
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PWM_PIN, 0);

  // Initialize motor in the stopped state
  stopMotor();
}

void loop() {
  // Read encoder value
  long encoderValue = encoder.getCount();

  // Print the encoder value
  Serial.println("Encoder Value: " + String(encoderValue));

  // You can add more code here to perform additional actions based on the encoder value

  delay(1000); // Adjust delay as needed
}

void stopMotor() {
  // Stop the motor by setting both IN3 and IN4 to LOW
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Set PWM to 0 for motor stop
  ledcWrite(0, 0);
}
