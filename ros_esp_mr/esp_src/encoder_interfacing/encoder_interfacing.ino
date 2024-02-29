
#include <ESP32Encoder.h>
#include <ros.h>
#include <std_msgs/Int32.h>

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
// Create encoder objects
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// Create ROS node handle
ros::NodeHandle nh;

// Create ROS publishers for encoder data
std_msgs::Int32 encoderLeftMsg;
ros::Publisher encoderLeftPub("encoder_data_left", &encoderLeftMsg);

std_msgs::Int32 encoderRightMsg;
ros::Publisher encoderRightPub("encoder_data_right", &encoderRightMsg);

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

  // Initialize motors in the stopped state
  stopMotors();

  // Initialize ROS
  nh.initNode();
  nh.advertise(encoderLeftPub);
  nh.advertise(encoderRightPub);
}

void loop() {
  // Read encoder values
  long encoderLeftValue = encoderLeft.getCount();
  long encoderRightValue = encoderRight.getCount();

  // Publish encoder values to ROS
  encoderLeftMsg.data = encoderLeftValue;
  encoderLeftPub.publish(&encoderLeftMsg);

  encoderRightMsg.data = encoderRightValue;
  encoderRightPub.publish(&encoderRightMsg);

  // Handle ROS communication
  nh.spinOnce();

  // You can add more code here to perform additional actions based on the encoder values
  // ...

  delay(1000); // Adjust delay as needed
}

void stopMotors() {
  // Stop the left motor by setting both IN1 and IN2 to LOW
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Stop the right motor by setting both IN3 and IN4 to LOW
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Set PWM to 0 for both motors
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}
