/*
 * SUCK - S.. Ultrasound Controller Keyboard
 *
 * Created by: ULTRASOUND GUYS (Craig Chalmers, Simon Fowler, Cailean MacLennan, William Tetlow)
 *
 * The code below uses a mixture of self-implemented and open-source libraries and functions.
 * Self-Implemented :
 *                - USSensors.h
 *                - doEncoderA()      - Interrupt driven
 *                - doEncoderB()      - Interrupt driven
 *                - doEncoderSwitch() - Interrupt driven
 *                - changeEncoderValue()
 *                - midiCreator(int cmd, int sensor, int distance)
 *                - setRGB(int x, int led)
 * Open-Source :
 *                - SegmentDisplay.h
 *                - ShiftPWM.h
 *
 * It provides the Arduino Ultrasound Harp with the functionality to perform as a MIDI controller.
 * Facilitating :
 *                - Using 8 ultrasonic sensors to detect a user's hand movements and position.
 *                - Visual user feedback on their playing style using linear interpolating RGB LED's. (RED -> BLUE -> GREEN)
 *                - Control over the type of musical scale played using a rotary encoder (RE).
 *                - Control over the type of modulation using the RE secondary function by pressing it inwards.
 *                    (Distinction between the RE's primary and secondary function is shown
 *                      by changing the colour of the RE's internal LED)
 *                - Visual user feedback over what type of scale and modulation selected using a 7-segment display.
 *                - Sending MIDI signals to an external VST through USB.
 *
 * We declare that all the work submitted for this project is our own work, unless otherwise stated
 * when using Arduino libraries.
 *
 * Signed : Craig Chalmers, Simon Fowler, Cailean MacLennan, William Tetlow.
 *
 */
#include <SegmentDisplay.h>
#include <USSensors.h>
//#define DEBUG

// Clock and data pins are pins from the hardware SPI, you cannot choose them yourself if you use the hardware SPI.
// Data pin is MOSI (Mega: 51)
// Clock pin is SCK (Mega: 52)
const int ShiftPWM_latchPin = 46;

// Common Anode requires this to be true
const bool ShiftPWM_invertOutputs = true;
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

// Set brightness levels, the update frequency and the number of shift registers.
// These values affect the load of ShiftPWM.
unsigned char maxBrightness = 255;
unsigned char pwmFrequency = 75;
int numRegisters = 3;
int numRGBleds = 8;

// MIDI DEFINITIONS
#define NOTE_OFF 0x80
#define NOTE_ON 0x90
#define MIDI_CC_CHANNEL 0xB2

// Encoder LEDs
#define blue 10
#define green 11
#define button 20

// MIDI Note Presets
int notePresets[6][8] = {
  {60, 62, 64, 65, 67, 69, 71, 72}, // C3 Major
  {60, 62, 63, 65, 67, 68, 70, 72}, // C3 Minor
  {60, 62, 64, 67, 69, 72, 74, 77}, // C3 Pentatonic
  {60, 63, 65, 67, 70, 72, 75, 77}, // C3 Minor Pentatonic
  {60, 63, 65, 66, 67, 70, 72, 75}, // C3 Blues
  {60, 62, 63, 65, 66, 68, 69, 71}  // C3 Diminished
};

// Pin values for the 8 US sensors
// Each array index corresponds to a sensor (0 -> 7)
int tPinValues[8] = {22, 24, 26, 28, 30, 32, 34, 36};
int ePinValues[8] = {23, 25, 27, 29, 31, 33, 35, 37};

// Pins for the segment display
int segmentPins[7] = {2, 3, 4, 5, 6, 8, 9};
USSensors sensors(tPinValues, ePinValues); // Instantiate Ultrasound sensors
SegmentDisplay segDisplay(segmentPins); // Instantiate the Seven Segment LED Display

// Rotary Encoder Variables
enum PinAssignments {
  encoderPinA = 19,   // rigth
  encoderPinB = 18,   // left
};
volatile unsigned int encoder_open_value = 0;
volatile unsigned int encoder_closed_value = 1;
volatile unsigned int encoder_switch_count = 0;

unsigned int lastReportedPos = 1;     // change management
static boolean rotating = false;      // debounce management
static boolean rotarySwitched = false;// rotary switch
static boolean switching = false;      // debounce switch

// encoder interrupt service routine vars
boolean A_set = false;
boolean B_set = false;
boolean sensorsPlaying[8];
boolean switch_set = false;

void setup() {
  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);
  // SetPinGrouping allows flexibility in LED setup.
  // If your LED's are connected like this: RRRRGGGGBBBBRRRRGGGGBBBB, use SetPinGrouping(4).
  ShiftPWM.SetPinGrouping(1); //This is the default, but I added here to demonstrate how to use the funtion
  ShiftPWM.Start(pwmFrequency, maxBrightness);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(button, INPUT);
  digitalWrite(button, HIGH);
  // Setup encoder pins as inputs
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  // encoder pin on interrupt 5 (pin 18)
  attachInterrupt(5, doEncoderA, CHANGE);
  // encoder pin on interrupt 4 (pin 19)
  attachInterrupt(4, doEncoderB, CHANGE);
  // encoder switch pin on interrupt 4 (pin 20)
  attachInterrupt(3, doEncoderSwitch, CHANGE);
  doEncoderSwitch();

  // MIDI requires baud rate of 31250
  #ifdef DEBUG
    Serial.begin(9600);
  #else
    Serial.begin (31250);
    for (int i = 0; i < 8; i++) {
      sensorsPlaying[i] = false;
    }
  #endif
}

// Interrupt on A changing state
void doEncoderA() {
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done
  // Test transition, did things really change?
  if ( digitalRead(encoderPinA) != A_set ) { // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if ( A_set && !B_set )
      changeEncoderValue(false); // false indicates incremented value
    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB() {
  if ( rotating ) delay (1);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
      changeEncoderValue(true); // true indicates decremented value
    rotating = false;
  }
}

// Interrupt on Switch changing state
void doEncoderSwitch() {
  // debounce
  if ( switching ) delay (1);  // wait a little until the bouncing is done
  // Test transition, did things really change?
  if ( digitalRead(button) != switch_set ) { // debounce once more
    switch_set = !switch_set;
    // adjust counter + if A leads B
    if ( switch_set ){
      // Change rotary encoder colour
      if (encoder_switch_count % 2)
      {
        segDisplay.turnOff();
        segDisplay.displayDigit(encoder_closed_value);
        digitalWrite(green, LOW);
        digitalWrite(blue, HIGH);
        rotarySwitched = true;
      }
      else
      {
        segDisplay.turnOff();
        segDisplay.displayDigit(encoder_open_value);
        digitalWrite(green, HIGH);
        digitalWrite(blue, LOW);
        rotarySwitched = false;
      }
      encoder_switch_count++;
    }
    rotating = false;  // no more debouncing until loop() hits again
  }
}

void changeEncoderValue(bool decrement) {
  int encMod;
  if (decrement) {
    encMod = -1;
  } else {
    encMod = 1;
  }
  if ( rotarySwitched ) {
    if ( encoder_closed_value + encMod > 0 && encoder_closed_value + encMod <= 8 ) {
      encoder_closed_value += encMod;
      encoder_closed_value = encoder_closed_value % 9;
      segDisplay.turnOff();
      segDisplay.displayDigit(encoder_closed_value);
    }
  }
  else {
    if ( encoder_open_value + encMod > 0 && encoder_open_value + encMod <= 6 ) {
      encoder_open_value += encMod;
      encoder_open_value = encoder_open_value % 7;
      segDisplay.turnOff();
      segDisplay.displayDigit(encoder_open_value);
    }
  }
}

void loop() {
  sensors.sendSignal();
  for (int i = 0; i < 8; i++) {
    //    Serial.print(sensors.sensorDistance[i]);
    //    Serial.print("\t");
    if (sensors.sensorDistance[i] != 0) {
      if (sensorsPlaying[i] == false) {
        sensorsPlaying[i] = true;
        noteCreator(NOTE_ON, i);
      } else {
        ccCreator(sensors.sensorDistance[i]);
      }
    } else {
      if (sensorsPlaying[i] == true) {
        sensorsPlaying[i] = false;
        noteCreator(NOTE_OFF, i);
      }
    }
  }
  for (int i = 0; i < 8; i++) {
    setRGBvalues(sensors.sensorDistance[i], i);
  }
}

void noteCreator(int cmd, int sensor) {
  int selectedScale[8];
  for (int i = 0; i < 8; i++) {
    selectedScale[i] = notePresets[encoder_open_value - 1][i];
  }
  Serial.write(cmd);
  Serial.write(selectedScale[sensor]);
  Serial.write(0x45);
}

void ccCreator(long distance) {
  byte channel = 0xB2 ; // 0xB0 is the first of 16 control channels
  long value = (127) * (distance - 25) / (200 - 25); // Convert the distance to scale 0-127
  int ccId;
  if (encoder_closed_value == 1) {
    ccId = 1;
  } else {
    ccId = 20 + encoder_closed_value - 2;
  }
  // Ensure we're between channels 1 and 16 for a CC message
  if (channel >= 0xB0 && channel <= 0xBF)
  {
#ifdef DEBUG
    Serial.print(channel);
    Serial.print(": ");
    Serial.println(value);
#elif defined(TEENSY_PLUS_PLUS) || defined(TEENSY_2) || defined(TEENSY_PLUS_PLUS_2)
    usbMIDI.sendControlChange(control, value, channel);
#else
    Serial.write(channel);
    Serial.write(ccId);
    Serial.write(value);
#endif
  }
}

void setRGBvalues(int x, int led) {
  //Calculates RGB vlues for current distance
  if (x != 0) {
    // Calculates fraction for RGB colours
    float fraction = ((200.00) * ((float)x - 25.00) / (200.00 - 25.00)) / 200;
    int g = 120;
    int r = 240;
    int value = (r - g) * fraction + g;
    ShiftPWM.SetHSV((((led % 8) - 7) * -1), value, 255, 255);
  } else {
    ShiftPWM.SetRGB((((led % 8) - 7) * -1), 255, 0, 0);
  }
}
