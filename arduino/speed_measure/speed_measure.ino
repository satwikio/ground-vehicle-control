// Define variables to store counts
int count0 = 0; // Counter for digitalRead() returning 0
int count1 = 0; // Counter for digitalRead() returning 1

// Variable to store previous state of pin 1
int previousState = LOW;

// Timer variables
unsigned long previousMillis = 0;   // Stores the last time the output was updated
const long interval = 500;         // Interval in milliseconds (2 seconds)

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(2, INPUT);  // Set digital pin 2 as input
}

void loop() {
  unsigned long currentMillis = millis();  // Get the current time

  int currentState = digitalRead(2); // Read digital pin 2
  
  // Check if pin state has changed
  if (currentState != previousState) {
    if (currentState == LOW) {
      count0++; // Increment count0 if current state is 0
    } else {
      count1++; // Increment count1 if current state is 1
    }
    previousState = currentState; // Update previous state
  }
  
  // Check if 2 seconds have elapsed
  if (currentMillis - previousMillis >= interval) {
    // Calculate the frequency over the 2-second interval
    float frequency = count0 + count1; // Average frequency in Hz
    float speed = frequency * 60.0/ (93.0 * 0.5); // Calculate speed in RPM (assuming 47 pulses per rotation)

    // Print the speed
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.println(" RPM");

    // Reset counts for the next interval
    count0 = 0;
    count1 = 0;
    previousMillis = currentMillis; // Save the current time for the next interval
  }
}
