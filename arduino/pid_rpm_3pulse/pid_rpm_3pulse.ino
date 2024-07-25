const int speedPin = 2; // Pin to measure the motor speed
const int motorPin = 3; // Pin to control the motor speed (PWM)
const int reversePin = 4; // Pin to control motor direction
const int brakePin = 5; // Pin to control motor brake
unsigned long startTime; // Variable to store the start time of the pulse
unsigned long endTime; // Variable to store the end time of the pulse
unsigned long highDuration; // Variable to store the duration of the high pulse
float rpm = 0; // Variable to store the calculated RPM

const float targetRPM1 = 200.0; // Desired RPM, can be positive or negative
const float targetRPM = fabs(targetRPM1); // Absolute value of the desired RPM
bool brake = false; // Variable to control the brake state

const float kp = 1.5; // Proportional gain for PID controller
const float ki = 0.5; // Integral gain for PID controller
const float kd = 0.0; // Derivative gain for PID controller (not used)
float integral = 0; // Integral term for PID controller
float error; // Current error between desired RPM and actual RPM
float previousError = 0; // Previous error for derivative term
unsigned long previousTime = 0; // Previous time for time difference calculation
float output; // Output value of the PID controller
int pwmValue = 0; // PWM value to control the motor (0-255 for analogWrite)

// Function to measure the pulse width of the speed signal
unsigned long getPulseWidth() {
  // Wait for the pin to go HIGH
  while (digitalRead(speedPin) == LOW);
  startTime = micros(); // Get the start time

  // Wait for the pin to go LOW, then HIGH multiple times to calculate duration
  while (digitalRead(speedPin) == HIGH);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == LOW);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == HIGH);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == LOW);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == HIGH);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == LOW);
  endTime = micros(); // Get the end time

  unsigned long highDuration1 = endTime - startTime; // Calculate the HIGH duration
  return highDuration1;
}

// Function to calculate the PWM value using a PID controller
int getPWM(float rpm) {
  error = targetRPM - rpm; // Calculate the current error
  if (rpm == 0) {
    output = 0.1 * error; // Simple proportional control if RPM is zero
  } else {
    integral += error * highDuration / 1000000; // Update integral term
    float derivative = 1000000 * (error - previousError) / highDuration; // Calculate derivative term
    previousError = error; // Update previous error
    output = kp * error + ki * integral + kd * derivative; // Calculate PID output
  }
  pwmValue = constrain(output, 0, 255); // Constrain the output to valid PWM range
  // constrainedOutput = (output, 0.0, 150.0);
  // if((constrainedOutput > output) && (signbit(error) == signbit(output))){
  //   ki = 0;
  // }
  // else{
  //   ki = 0.5;
  // }
}

// Function to write the PWM value to the motor and calculate the new RPM
void writePWM() {
  getPWM(rpm); // Calculate the new PWM value
  analogWrite(motorPin, pwmValue); // Write the PWM value to the motor
  highDuration = getPulseWidth(); // Calculate the HIGH duration of the pulse
  rpm = (0.34*3920000) / highDuration; // Calculate the RPM based on pulse duration
  //Serial.println(rpm); // Uncomment to print the RPM to the Serial Monitor
}

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(speedPin, INPUT); // Set the speed pin as input
  pinMode(motorPin, OUTPUT); // Set the motor control pin as output
  pinMode(reversePin, OUTPUT); // Set the reverse control pin as output
  pinMode(brakePin, OUTPUT); // Set the brake control pin as output
}

void loop() {
  if (brake == false) { // If the brake is not engaged
    digitalWrite(brakePin, LOW); // Disable the brake
    if (targetRPM1 > 0) {
      digitalWrite(reversePin, HIGH); // Set motor direction to forward
      writePWM(); // Update motor speed
    } else if (targetRPM1 < 0) {
      digitalWrite(reversePin, LOW); // Set motor direction to reverse
      writePWM(); // Update motor speed
    }
  } else if (brake == true) { // If the brake is engaged
    digitalWrite(brakePin, HIGH); // Enable the brake
  }
}
