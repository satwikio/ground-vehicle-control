const int pin = 2; // Pin to read the pulses
unsigned long highDuration; // Variable to store the duration of HIGH pulse
unsigned long lowDuration;  // Variable to store the duration of LOW pulse
float highSpeed;
float lowSpeed;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
}

void loop() {
  // Measure the duration of the HIGH pulse
  highDuration = pulseIn(pin, HIGH);
  highSpeed = 21600000/(47*highDuration);
  
  // Measure the duration of the LOW pulse
  lowDuration = pulseIn(pin, LOW);
  lowSpeed = 216000/(47*lowDuration);
  
  // Print the durations to the Serial Monitor
  Serial.print("HIGH duration: ");
  //Serial.print(highDuration);
  Serial.print(highSpeed);
  Serial.println(" microseconds");
  
  Serial.print("LOW duration: ");
  //Serial.print(lowDuration);
  Serial.print(lowSpeed);
  Serial.println(" microseconds");
  
  // Add a small delay to avoid spamming the serial monitor
  //delay(100);
}
