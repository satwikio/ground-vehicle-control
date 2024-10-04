#include <Servo.h>  // Include the Servo library
Servo myServo;  // Create a Servo object

#define PPM_PIN 2
#define NUM_CHANNELS 4

#define MIN_CHANNEL_PULSE 1000
#define MAX_CHANNEL_PULSE 2000
#define SYNC_PULSE 3000

// Variables for PPM receiver
volatile uint16_t channelValues[NUM_CHANNELS];
volatile uint8_t currentChannel = 0;
volatile bool newFrame = false;

// Timer and PPM signal capture
void ppmInterrupt() {
    static unsigned long lastTime = 0;
    unsigned long time = micros();
    unsigned long pulseLength = time - lastTime;
    lastTime = time;

    if (pulseLength > SYNC_PULSE) {  
        currentChannel = 0;
        newFrame = true;
    } else if (pulseLength >= MIN_CHANNEL_PULSE && pulseLength <= MAX_CHANNEL_PULSE && currentChannel < NUM_CHANNELS) {
        channelValues[currentChannel] = pulseLength;
        currentChannel++;
    }
}

void setup() {
    Serial.begin(2000000);

    myServo.attach(8);  // Attach the servo to digital pin 9
    
    // Initialize the channel values
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        channelValues[i] = 1500;
    }

    // Attach interrupt to capture PPM signal
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, RISING);
}

float mapChannel(uint16_t pulseWidth, float minVal, float maxVal) {
    return (float(pulseWidth) - 1000) / 1000 * (maxVal - minVal) + minVal;
}

volatile uint16_t makeZero(volatile uint16_t val, int limit){
  if (abs(val) - 1500 < limit){
    val = 1500;
    return val; 
  }
  else{ return val;}
}



void loop() {
    if (newFrame) {
        newFrame = false;
        
        // Disable interrupts while reading values
        noInterrupts();
        uint16_t currentValues[NUM_CHANNELS];
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            currentValues[i] = channelValues[i];
        }
        interrupts();
        
        // Mapping channels to respective variables
        float angle = mapChannel(makeZero(currentValues[3], 30), 0, 180);
//        Serial.println(angle);
        myServo.write(round(angle));
        Serial.println(round(angle) );
        
    }


}
