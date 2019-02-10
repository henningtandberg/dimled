/** File:   dimled.ino
 *  Group:  AUX - IN5120
 */

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define LED_PIN     6       // Control pin for led strip?
#define NUM_PIXELS  14      // Number of pixels in strip?
#define INC_VAL     42.5    // The amount of increments per dim cycle
#define MIC_PIN     A0      // Sample pin for microphone
#define SAMP_WIN    50      // Sample window width in ms (50 ms = 20Hz)
#define LOOP_DELAY  50      // Amount of ms delay per main iteration
#define VTHRESH     3.0     // Above this value the lights dim up

int colStrBlue = 50;        // Color strength of blue light
double voltsMax = 0.0;      // Used to measure increasing sound.

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {

    #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
    #endif
    
    /* Initialize the led strip */
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
    double volts = (peak2Peak * 4.9) / 1024;
    
    Serial.println("Volts from peak2peakSample: "+ (String)volts);
       
    /* Increments when above thresh hold? */
    if (volts >= VTHRESH) {
        incGradient();
    } else {
        dimGradient();
    }
  
    strip.show();
    delay(LOOP_DELAY);
}

/** dimGradient() - Gradually dims the leds of the strip one by one.
 */
void dimGradient() {
    int newColStrBlue = colStrBlue - INC_VAL;

    if (newColStrBlue >= 0) {
        for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(0, 0, newColStrBlue));
        }
        colStrBlue = newColStrBlue;
    }
}


/** incGradient() - Gradually increments the leds of the strip one by one.
 */
void incGradient() {
    int newColStrBlue = colStrBlue + INC_VAL;
 
    if (newColStrBlue <= 255) {
        for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(0, 0, newColStrBlue));
        }
        colStrBlue = newColStrBlue;
    }
}

/** samplePeak2PeakAmp() - Calculates the peak to peak amplitude.
 *  @sampleWindow: Amount of ms to sample from mic.
 *  @micPin: Analog pin to sample from.
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
