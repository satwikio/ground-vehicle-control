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

    // Initialize the channel values
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        channelValues[i] = 1500;
    }

    // Attach interrupt to capture PPM signal
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, RISING);
}

// Map the PPM channel values (1000-2000) to a range of minVal to maxVal
float mapChannel(uint16_t pulseWidth, float minVal, float maxVal) {
    return (float(pulseWidth) - 1000) / 1000 * (maxVal - minVal) + minVal;
}

float correctAlpha(float val){
  if (abs(val)>1.57){
    if(val>0){val = 3.14 - val;}
    else{val = val + 3.14;}
  }
  return constrain(val, -1.57, 1.57);
}

volatile uint16_t makeZero(volatile uint16_t val, int limit){
  if (abs(val) - 1500 < limit){
    val = 1500;
    return val; 
  }
  else{ return val;}
}

float getSign(float val){
  if(val != 0){
    return val/abs(val);
  }
  else {return 0;}
}

float correctV(float vel, float vx, float vy){
  if(vx <0){
    vel = -1*vel; 
  }
  else if(vx ==0 && vy){
    vel = -1*vel; 
  }
  return vel;
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
        float vx = mapChannel(makeZero(currentValues[1],40), -1.0, 1.0);  // Channel 2 (0-based index 1)
        float vy = mapChannel(makeZero(currentValues[0],30), -1.0, 1.0);  // Channel 1 (0-based index 0)
        float omega = mapChannel(makeZero(currentValues[3],30), -1.0, 1.0); // Channel 4 (0-based index 3)

        // Calculate v and alpha
        float v = correctV(sqrt(vx * vx + vy * vy), vx , vy);    // Velocity magnitude
        float alpha = correctAlpha(atan2(vy, vx));          // Orientation angle

        // Printing values to the serial monitor for debugging
        Serial.print("vx: "); Serial.println(vx);
        Serial.print("vy: "); Serial.println(vy);
        Serial.print("v: "); Serial.println(v);
        Serial.print("alpha: "); Serial.println(alpha);
        Serial.print("omega: "); Serial.println(omega);
    }

    delay(1000);  // Add delay to avoid flooding serial
}
