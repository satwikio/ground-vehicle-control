// Define variables to store counts
int count0 = 0; // Counter for digitalRead() returning 0
int count1 = 0; // Counter for digitalRead() returning 1
float controlSignal;
 float error;
const float sampleTime = 0.08;  //time in seconds

// Variable to store previous state of pin 2
int previousState = LOW;

// Timer variables
unsigned long previousMillis = 0;   // Stores the last time the output was updated
const long interval = 600;          // Interval in milliseconds (500 milliseconds)

// Desired speed in RPM
const float targetRPM = 80.0;

// PID control variables
float Kp = 2.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 5.0;  // Derivative gain

float previousError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(2, INPUT);  // Set digital pin 2 as input
  pinMode(3, OUTPUT); // Set digital pin 3 as output (PWM)
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
  
  // Check if 500 milliseconds have elapsed
  if (currentMillis - previousMillis >= interval) {
    // Calculate the frequency over the 500ms interval
    float frequency = (count0 + count1); // Frequency in Hz
    float rpm = frequency * 60.0 / (94.0 * sampleTime); // Calculate RPM (assuming 47 pulses per rotation)

    // Calculate the error
    error = targetRPM - rpm;

    // Calculate the integral term
    integral += error * (interval / 1000.0);

    // Calculate the derivative term
    float derivative = -(error - previousError) / (interval / 1000.0);

    // Calculate the control signal
    controlSignal = Kp * error + Ki * integral + Kd * derivative;

    // Limit control signal to PWM range
    controlSignal = constrain(controlSignal, 0, 100);

    // Output the control signal to the motor
    //analogWrite(3, controlSignal);

    // Print the RPM and control signal
    Serial.print("RPM: ");
    Serial.println(rpm);
    Serial.print(" | Control Signal (PWM): ");
    Serial.println(controlSignal);

    // Reset counts for the next interval
    count0 = 0;
    count1 = 0;
    previousMillis = currentMillis; // Save the current time for the next interval

    // Update the previous error
    previousError = error;
  }
  analogWrite(3, controlSignal);
  //Serial.println(error);
}
