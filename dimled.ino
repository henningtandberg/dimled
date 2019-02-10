/** Author: ...
 *  File:   dimled.ino
 *  Group:  AUX - IN5120
 */

#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

/** Debug flag */
#define DEBUG       1       // Set to 0 in production

/** Arduino pins */
#define LED_PIN     6       // Control pin for led strip?
#define MIC_PIN     A0      // Sample pin for microphone

/** Configurations values */
#define NUM_PIXELS  17      // Number of pixels in strip
#define INC_VAL     32      // The amount of increments per pixel per cycle
#define DEC_VAL     32      // The amount of decrements per pixel per cycle
#define VTHRESH     2.0     // Above this value and the lights dim up
#define VTHRESH_MAX 3.3     // Maximum volts from mic
#define SAMP_WIN    50      // Sample window width in ms (50 ms = 20Hz)
#define LOOP_DELAY  10      // Amount of ms delay per main iteration

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {

#if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
    
    /* Initialize the NeoPixel LED strip */
    strip.begin();
    for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0));
    }
    strip.show();

    Serial.begin(9600);
}

void loop() {
      
    /* Sample mic and get volt of peak2peak amplitude */
    int peak2Peak = samplePeak2PeakAmp(SAMP_WIN, MIC_PIN);
    double volts = (peak2Peak * 5.) / 1024;
    
#if (DEBUG == 1) 
    Serial.println("Volts from peak2peakSample: "+ (String)volts);
#endif       

    /* Increments when above thresh hold? */
    if (volts >= VTHRESH) {
        incGradient(mapFloat(volts, VTHRESH, VTHRESH_MAX, 1.5, 3.0));
    } else {
        dimGradient();
    }
  
    strip.show();
    delay(LOOP_DELAY);
}

/** dimGradient() - Gradually dims the leds of the strip one by one.
 */
void dimGradient() { 
    for (int i = strip.numPixels() - 1; i >= 0; i--) {
        uint32_t value = strip.getPixelColor(i) & 0xFF;
        
        if (value > 0) {
            int newValue = value - DEC_VAL;
            if (newValue < 0)
                newValue = 0;
            
            strip.setPixelColor(i, strip.Color(0, 0, newValue));
            break;
        }
    }
}


/** incGradient() - Gradually increments the leds of the strip one by one.
 */
void incGradient(double acceleration) {
#if (DEBUG == 1)
    Serial.println("Acceleration: " + (String)acceleration);
#endif

    for (int i = 0; i < strip.numPixels(); i++) {
        uint32_t value = strip.getPixelColor(i) & 0xFF;
        
        if (value < 255) {
            int newValue = (value + INC_VAL) * (int)acceleration;
            if (newValue > 255)
                newValue = 255;

            strip.setPixelColor(i, strip.Color(0, 0, newValue));
            break;
        }
    }
}

/** samplePeak2PeakAmp() - Calculates the peak to peak amplitude.
 *  @sampleWindow:  Amount of ms to sample from mic.
 *  @micPin:        Analog pin to sample from.
 *
 *  Return: The peak to peak amplitude from the sample window.
 */
int samplePeak2PeakAmp(int sampleWindow, int micPin) {
    unsigned long startMillis = millis();
    unsigned int signalMin = 1024;
    unsigned int signalMax = 0;
    unsigned int sample = 0;

    while (millis() - startMillis < sampleWindow) {
        sample = analogRead(micPin);
        
        if (sample < 1024) {
            if (sample > signalMax) {
                signalMax = sample;
            } else if (sample < signalMin) {
                signalMin = sample;
            }
        }
    }
   
    return signalMax - signalMin; // max - min = peak-peak amplitude
}

/** mapFloat() - Maps a value to a range of numbers.
 *  @x:         Value to map
 *  @inMin:     Min value of x
 *  @inMax:     Max value of x
 *  @outMin:    Min value of range
 *  @outMax:    Max value of range
 *
 *  Return: A value within the specified range.
 */
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
    return (float)(x - inMin) * (outMax - outMin) / (float)(inMax - inMin) + outMin;
}
