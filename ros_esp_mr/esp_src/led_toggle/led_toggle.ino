#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void messageCb(const std_msgs::Empty& toggle_msg) {
    if(digitalRead(2)== HIGH) {
        delay(2000);
        digitalWrite(2,0);
    }
    else{
        digitalWrite(2,1);
    }
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb);

void setup() {
  pinMode(2, OUTPUT);  // use pin 2 for the built-in LED on ESP32
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
