const int irSensorPin = 14; // Pin number where the IR sensor is connected

void setup() {
  pinMode(irSensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.println("Checking...");

  int irSensorValue = digitalRead(irSensorPin);

  if (irSensorValue == HIGH) {
    Serial.println("Object detected!");
  } else {
    Serial.println("No object detected.");
  }

  delay(1000); // Adjust delay as needed
}
