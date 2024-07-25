const int pin = 3; // Pin to measure
unsigned long startTime;
unsigned long startTime1;
unsigned long startTime2;
unsigned long endTime;
unsigned long endTime3;
unsigned long endTime2;
unsigned long highDuration;
unsigned long endTime1;
float rpm;

void setup() {
  Serial.begin(250000); // Initialize serial communication
  pinMode(pin, INPUT); // Set pin as input
}

void loop() {
  // Wait for the pin to go HIGH
  while (digitalRead(pin) == LOW);
  startTime = micros(); // Get the start time

  // Wait for the pin to go LOW
  while (digitalRead(pin) == HIGH);
  endTime = micros(); // Get the end time

  while (digitalRead(pin) == LOW);
  endTime1 = micros(); // Get the end time

  // Wait for the pin to go LOW
  while (digitalRead(pin) == HIGH);
  endTime1 = micros(); // Get the end time

  while (digitalRead(pin) == LOW);
  endTime2 = micros(); // Get the end time

  // Wait for the pin to go LOW
  while (digitalRead(pin) == HIGH);
  endTime2 = micros(); // Get the end time

  while (digitalRead(pin) == LOW);
  endTime3 = micros(); // Get the end time

  highDuration = endTime3 - startTime; // Calculate the HIGH duration
  rpm = 1321860/highDuration;   

  //Serial.print("High Duration: ");
  Serial.println(rpm);
  //Serial.println(" microseconds");

  //delay(1000); // Wait for 1 second before measuring again
}
