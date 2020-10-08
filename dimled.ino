
/** File:   dimled.ino
 *  
 *  Code for group AUX - IN5120 - Spring 2019.
 */

#include <Adafruit_NeoPixel.h>
#include <L293DDriver.h>

/** Set to 0 when in production */
#define DEBUG               1   //
/** Debug printing "function"   */
#define DEBUGPRINT(s)           \
    do {                        \
        if (DEBUG) {            \
            Serial.println s ;  \
        }                       \
    } while(0)

/** Enable code */
#define PILLOW_ENABLED      0   //
#define MOTOR_ENABLED       0   //
#define BUBBLES_ENABLED     1   //
#define TENTACLES_ENABLED   0   //

/** Configuration values for motor
 */
#define MOT1_PWM_PIN        11  //  PWM pin of motor
#define MOT2_PWM_PIN        10  //  UNUSED
#define MOT1_IN_PIN1        9   //  Control pin 1 of motor
#define MOT1_IN_PIN2        8   //  Control pin 2 of motor
#define MOT2_IN_PIN1        7   //  UNUSED
#define MOT2_IN_PIN2        6   //  UNUSED
#define MOT_SPEED           255 //  Speed of motor

/** Configuration values for bubble strip
 */
#define BUBB_DATA_PIN       5   // Data pin for bubble led strip
#define BUBB_NUM_PIXELS     17  // Number of pixels in strip
#define BUBB_INC_VAL        32  // The amount of increments per pixel per cycle
#define BUBB_DEC_VAL        32  // The amount of decrements per pixel per cycle

/** Configuration values for tentacle strips with 30 LEDs
 */
#define TENT30_DATA_PIN     4   // Data pin for 30 LED tentacle strip
#define TENT30_NUM_STRIPS   4   // Number of 30-LED strips
#define TENT30_NUM_PIXELS   30  // Number of pixels in strip(s)
#define TENT30_CHS_WIDTH    5   // Chase width in pixels

/** Configuration values for tentacle strips with 30 LEDs
 */
#define TENT60_DATA_PIN     3   // Data pin for 60 LED tentacle strip
#define TENT60_NUM_STRIPS   4   // Number of 60-LED strips
#define TENT60_NUM_PIXELS   60  // Number of pixels in strip(s)
#define TENT60_CHS_WIDTH    5   // Chase width on pixels

/** Configuration values for mic 
 */
#define MIC_PIN             A0  // Sample pin for microphone
#define MIC_VTHRESH         2.0 // Above this value and the lights dim up
#define MIC_VTHRESH_MAX     3.3 // Maximum volts from mic
#define MIC_SAMP_WIN        50  // Sample window width in ms (50 ms = 20Hz)

/** Configuration values for pillow
 */
#define PILLOW_PIN          A1  // Pillow Pin
#define PILLOW_THRESH       40  // Threshold for activating the pillow

/** Configuration values for other things
 */
#define LOOP_DELAY          10  // Amount of ms delay per main iteration

Adafruit_NeoPixel bubbStrip = Adafruit_NeoPixel(BUBB_NUM_PIXELS, BUBB_DATA_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel tentStrips30[TENT30_NUM_STRIPS] = {
    Adafruit_NeoPixel(TENT30_NUM_PIXELS, TENT30_DATA_PIN, NEO_GRB + NEO_KHZ800)
};

Adafruit_NeoPixel tentStrips60[TENT60_NUM_STRIPS] = {
    Adafruit_NeoPixel(TENT60_NUM_PIXELS, TENT60_DATA_PIN, NEO_GRB + NEO_KHZ800)
};

L293DDriver driver(MOT1_IN_PIN1, MOT1_IN_PIN2, MOT2_IN_PIN1, MOT2_IN_PIN2,
MOT1_PWM_PIN, MOT2_PWM_PIN);
L293DDCMotor *motor = NULL;
/**
 *  SETUP
 */
void setup() {
 
#if BUBBLES_ENABLED == 1
    /* Initialize the Bubble NeoPixel LED strip */
    bubbStrip.begin();
    stripSetAll(&bubbStrip, 0, 0, 0);
#endif

#if TENTACLES_ENABLED == 1
    /* Initialize the Tentacle NeoPixel LED strip */
    for (int i = 0; i < TENT30_NUM_STRIP; i++) {
        tentStrips30[i].begin();
        stripSetAll(&tentStrips30[i], 0, 0, 0);
    }
    
    for (int i = 0; i < TENT60_NUM_STRIPS; i++) {
        tentStrips60[i].begin();
        stripSetAll(&tentStrips60[i], 0, 0, 0);
    } 
#endif

#if PILLOW_ENABLED == 1
    /* Initialize the pillow */
    pinMode(PILLOW_PIN, INPUT_PULLUP);
#endif

#if MOTOR_ENABLED == 1 
    /* Initialize the motor */
    driver.begin();
    motor = driver.getDCMotor(M1);
    motor->setSpeed(MOT_SPEED);
#endif

#if DEBUG == 1    
    Serial.begin(9600);
#endif
}

/**
 *  MAIN LOOP
 */
void loop() {
         
#if PILLOW_ENABLED == 1
    int pillowValue = analogRead(PILLOW_PIN);
    
    if (pillowValue > PILLOW_THRESH) {
#endif
    
#if TENTACLES_ENABLED == 1
        tentCrossFade(Blue, Pink, 20);
        tentCrossFade(Pink, Blue, 20);
#endif   

#if MOTOR_ENABLED == 1 
    //TEST CODE
    int d = 1000; 
    DEBUGPRINT(("DRIVE FORWARDS FOR: " + (String)d + " Speed: " + (String)MOT_SPEED));
    motor->drive(DIR_FWD);
    delay(6000);
    //motor->drive(DIR_STOP);

    DEBUGPRINT(("WAIT 5 sec"));
    //delay(5000);    

    DEBUGPRINT(("DRIVE BACKWARDS FOR: " + (String)d + " Speed: " + (String)MOT_SPEED));
    motor->drive(DIR_FWD);
    delay(5000);
    //motor->drive(DIR_STOP);

    DEBUGPRINT(("WAIT 5 sec"));
    //delay(5000);
#endif

#if BUBBLES_ENABLED == 1
        /* Sample mic and get volt of peak2peak amplitude */
        int peak2Peak = samplePeak2PeakAmp(MIC_SAMP_WIN, MIC_PIN);
        double volts = (peak2Peak * 5.) / 1024;
     
        DEBUGPRINT(("Volts from peak2peakSample: "+ (String)volts));

        /* Increments when above thresh hold? */
        if (volts >= MIC_VTHRESH) {
            bool top = bubbIncrement(mapFloat(volts, MIC_VTHRESH, MIC_VTHRESH_MAX, 1.5, 3.0)); 
        } else {
            bool bot = bubbDecrement();
        }
#endif

#if PILLOW_ENABLED == 1
    }
#endif
  
    delay(LOOP_DELAY);
}

/**
 *  Motor code 
 */

/**
 */
void stripSetAll(Adafruit_NeoPixel *strip, uint8_t red, uint8_t green, uint8_t blue) {
    for (int i = 0; i < strip->numPixels(); i++) {
        strip->setPixelColor(i, red, green, blue);
    }
    
    strip->show();
}

/**
 *  Bubble code
 */

/** bubbDecrement() - Gradually dims the leds of the strip one by one.
 */
bool bubbDecrement() {
    int newValue = 0;
    int i;


     for (i = 1; i < bubbStrip.numPixels(); i++) {
        uint32_t value = bubbStrip.getPixelColor(i) & 0xFF;
        
        if (value > 0) {
            newValue = (value - BUBB_DEC_VAL);
            
            if (newValue < 0) newValue = 0;
            
            bubbStrip.setPixelColor(i, 0, 0, (uint8_t)newValue & 0xFF);
            break;
        }
    }
    
    bubbStrip.show();
    return (newValue == 0 && i == (BUBB_NUM_PIXELS));
}

/** bubbIncrement() - Gradually increments the leds of the strip one by one.
 *  @acceleration:  Acceleration of dim
 */
bool bubbIncrement(double acceleration) {
    int newValue = 255;
    int i;

    DEBUGPRINT(("Acceleration: " + (String)acceleration));

    for (i = bubbStrip.numPixels() - 1; i >= 1; i--) {
        uint32_t value = bubbStrip.getPixelColor(i) & 0xFF;
        
        if (value < 255) {
            newValue = (value + BUBB_INC_VAL) * (int)acceleration;
            
            if (newValue > 255) newValue = 255;

            bubbStrip.setPixelColor(i, 0, 0, (uint8_t)newValue & 0xFF);
            break;
        }
    }

    bubbStrip.show();
    return (newValue == 255 && i == 1);
}

/**
 *  MIC code
 */

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

/**
 *  Tentacle code
 */

/**
 */
/*void tentChase(uint8_t red, uint8_t green, uint8_t blue, uint32_t delayTime) {
    for (int i = 0; i < tentStrip.numPixels() + TENT_CHS_WIDTH; i++) {
        tentStrip.setPixelColor(i, red, green, blue);
        tentStrip.setPixelColor(i - TENT_CHS_WIDTH, 0);
        tentStrip.show();
        delay(delayTime);
    }
}*/

/**
 */
void tentCrossFade(uint32_t startColor, uint32_t endColor, unsigned long speed) {
    uint8_t startRed = (startColor >> 16) & 0xFF;
    uint8_t startGreen = (startColor >> 8) & 0xFF;
    uint8_t startBlue = startColor & 0xFF;
    
    uint8_t endRed = (endColor >> 16) & 0xFF;
    uint8_t endGreen = (endColor >> 8) & 0xFF;
    uint8_t endBlue = endColor & 0xFF;

    for (int i = 0; i < 256; i++) {
        uint8_t red = map(i, 0, 255, startRed, endRed);
        uint8_t green = map(i, 0, 255, startGreen, endGreen);
        uint8_t blue = map(i, 0, 255, startBlue, endBlue);
        
        for (int j = 0; j < TENT30_NUM_STRIPS; i++) {
            stripSetAll(&tentStrips30[i], red, green, blue);
        }
        
        for (int j = 0; j < TENT60_NUM_STRIPS; i++) {
            stripSetAll(&tentStrips60[i], red, green, blue);
        }
    }
}
