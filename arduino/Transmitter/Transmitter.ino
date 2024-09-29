


#define PPM_READER

#include <Arduino.h>
#include <InterruptHandler.h>


class PPMReader : InterruptHandler {

    public:
    
    // The range of a channel's possible values
    unsigned long minChannelValue = 1000;
    unsigned long maxChannelValue = 2000;

    /* The maximum error (in either direction) in channel value
     * with which the channel value is still considered valid */
    unsigned long channelValueMaxError = 10;

    /* The minimum value (time) after which the signal frame is considered to
     * be finished and we can start to expect a new signal frame. */
    unsigned long blankTime = 2100;


    private:

    // The pin from which to listen for interrupts
    byte interruptPin = 0;

    // The amount of channels to be expected from the PPM signal.
    byte channelAmount = 0;

    // Arrays for keeping track of channel values
    volatile unsigned long *rawValues = NULL;
    volatile unsigned long *validValues = NULL;

    // A counter variable for determining which channel is being read next
    volatile byte pulseCounter = 0;

    // A time variable to remember when the last pulse was read
    volatile unsigned long microsAtLastPulse = 0;


    public:

    PPMReader(byte interruptPin, byte channelAmount);
    ~PPMReader();

    /* Returns the latest raw (not necessarily valid) value for the
     * channel (starting from 1). */
    unsigned long rawChannelValue(byte channel);

    /* Returns the latest received value that was considered valid for the channel (starting from 1).
     * Returns defaultValue if the given channel hasn't received any valid values yet. */
    unsigned long latestValidChannelValue(byte channel, unsigned long defaultValue);

    private:

    // An interrupt service routine for handling the interrupts activated by PPM pulses
    virtual void handleInterrupt(int8_t interruptNum);

};
#include <PPMReader.h>
// #include <InterruptHandler.h>   <-- You may need this on some versions of Arduino

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
int interruptPin = 3;
int channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Print latest valid values from all channels
    for (int channel = 1; channel <= channelAmount; ++channel) {
        unsigned long value = ppm.latestValidChannelValue(channel, 0);
        Serial.print(String(value) + " ");
    }
    Serial.println();
}
