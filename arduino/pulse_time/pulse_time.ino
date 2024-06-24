const int pin = 2; // Pin to measure
unsigned long startTime;
unsigned long endTime;
unsigned long highDuration;
unsigned long highDuration1;
float rpm;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(pin, INPUT); // Set pin as input
}

unsigned long getPulseWidth() {
  // Wait for the pin to go HIGH
  while (digitalRead(pin) == LOW);
  startTime = micros(); // Get the start time

  // Wait for the pin to go LOW
  while (digitalRead(pin) == HIGH);
  endTime = micros(); // Get the end time

  highDuration = (endTime - startTime); // Calculate the HIGH duration
  return highDuration;
}

void loop() {
  
  if(digitalRead(pin) == LOW){
    highDuration1 = getPulseWidth();
  }
  rpm = 600000/highDuration1;
  Serial.println(rpm);
}
