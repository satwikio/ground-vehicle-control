// Define variables to store counts
int count0 = 0; // Counter for digitalRead() returning 0
int count1 = 0; // Counter for digitalRead() returning 1

// Variable to store previous state of pin 1
int previousState = LOW;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(2, INPUT);  // Set digital pin 1 as input
}

void loop() {
  int currentState = digitalRead(2); // Read digital pin 1
  
  // Check if pin state has changed
  if (currentState != previousState) {
    if (currentState == LOW) {
      count0++; // Increment count0 if current state is 0
    } else {
      count1++; // Increment count1 if current state is 1
    }
    previousState = currentState; // Update previous state
  }
  
  // Print the counts
  Serial.print("Count of 0: ");
  Serial.print(count0);
  Serial.print(", Count of 1: ");
  Serial.println(count1);
}
