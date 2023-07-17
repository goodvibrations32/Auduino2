// Auduino2, the Lo-Fi granular synthesiser, written for Control Voltage / Gate signals, including sending itself Gate.
//
// Original Auduino by Peter Knight, Tinker.it http://tinker.it (defunct)
// Help:      http://code.google.com/p/tinkerit/wiki/Auduino
// More help: http://groups.google.com/group/auduino

// modified by Joshua A.C. Newman http://glyphpress.com
// to add CV/Gate, reorganize the pins, add triggering, and add more explicit comments.
// GitHub project: https://github.com/JoshuaACNewman/Auduino2
//
// THE PINS:
// These two work together to make the first of two waveforms:
// Analog in 0: Grain 1 pitch.
// Analog in 1: Grain 2 decay.
//
// These two work together to make the second of two waveforms:
// Analog in 2: Grain 1 decay.
// Analog in 3: Grain 2 pitch.
//
// Analog in 4: Grain repetition frequency. At the bottom of the range, this is BPM. Above a certain level, it's pitch.
//
// Digital 3: Audio out (Digital 11 on ATmega8). This pin is being accessed directly and affected by the timers.
// To use for outgoing voltage control, you may need to add a voltage follower circuit
// to avoid drawing too much current and affecting the signal.
//
// Digital 2: Gate input. This triggers the synthesizer.
//            Connect it to 5v permanently to always play, with a switch to switch between Gate and constant play, or
//
// VOLTAGE CONTROL
// For incoming Control Voltage, attach a vactrol to each potentiometer.
// Optionally, install a three-pole switch between the incoming legs of the vactrol and either side of the pot to allow for inverting the CV.
//
// One channel of a TRS cable goes to the vactrol and one to the GATE pin.
//
// Changelog:
// 19 Nov 2008: Added support for ATmega8 boards
// 21 Mar 2009: Added support for ATmega328 boards
// 7 Apr 2009: Fixed interrupt vector for ATmega328 boards
// 8 Apr 2009: Added support for ATmega1280 boards (Arduino Mega)
// 8 Nov 2018: Reorganized pins, added comments, added Gate (Trigger) input, functionalized and abstracted

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;
uint16_t grain2PhaseAcc;
uint16_t grain2PhaseInc;
uint16_t grain2Amp;
uint8_t grain2Decay;

// To retrigger the grains when Gate first gets pulled HIGH, we need to know if the sound has been just trailing off.
boolean triggerGate;

// Map Analogue channels
#define SYNC_CONTROL         (0)  // BPM or pitch, depending on how high you turn it. 
// To control pitch with CV while using a tone-frequency Sync value, plug in here.

#define GRAIN_FREQ_CONTROL   (1)  // Frequency of grain 1
#define GRAIN_DECAY_CONTROL  (2)  // Decay rate of grain 1

#define GRAIN2_FREQ_CONTROL  (3)  // Frequency of grain 2
#define GRAIN2_DECAY_CONTROL (4)  // Decay rate of grain 2

#define GATE_CONTROL         (2)  // Half of CV/Gate. Connect the Gate channel of all your inputs to this pin.
// Control Voltage should go to individual parameters.
// If you want to voltage-control pitch, connect it to the SYNC_CONTROL pin.

// PIN SELECTION BY BOARD
// Compile-time selectiopn of pins by µC/development board specs:
// Changing these will also requires rewriting audioOn().

//
// On old ATmega8 boards.
//    Output is on pin 11
//
/* #define LED_PIN       13  // Use for outgoing Gate signal. */
/* #define LED_PORT      PORTB */
/* #define LED_BIT       5 */
/* #define PWM_PIN       11 */
/* #define PWM_INTERRUPT TIMER2_OVF_vect */
/* // */
/* // On the Arduino Mega */
/* //    Output is on pin 3 */
/* // */
/* #define LED_PIN       13  // Use for outgoing Gate signal. */
/* #define LED_PORT      PORTB */
/* #define LED_BIT       7 */
/* #define PWM_PIN       3 */
/* #define PWM_INTERRUPT TIMER3_OVF_vect */
/* // */
/* // For modern ATmega168 and ATmega328 boards */
/* //    Output is on pin 3 */
/* // */
int OCR2B = 50;
#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define LED_PIN       13  // Use for outgoing Gate signal.
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_INTERRUPT __vector_TIMER2_OVF
/*  SYNC FREQUENCY MAPPING
    Select which mapping you use at runtime in grainBuild().
    Values sound like a beat under ~10, and tonal from about 10 up.
*/

// Smooth logarithmic mapping
//
uint16_t antilogTable[] = {
  0xfd3e, 0xfa84, 0xf7d1, 0xf525, 0xf281, 0xefe5, 0xed4f, 0xeac1, 0xe839, 0xe5b9,
  0xe340, 0xe0cd, 0xde61, 0xdbfc, 0xd99d, 0xd745, 0xd4f3, 0xd2a8, 0xd063, 0xce25,
  0xcbec, 0xc9ba, 0xc78d, 0xc567, 0xc347, 0xc12c, 0xbf18, 0xbd09, 0xbaff, 0xb8fc,
  0xb6fe, 0xb505, 0xb312, 0xb124, 0xaf3b, 0xad58, 0xab7a, 0xa9a1, 0xa7ce, 0xa5ff,
  0xa435, 0xa270, 0xa0b0, 0x9ef5, 0x9d3f, 0x9b8d, 0x99e0, 0x9838, 0x9694, 0x94f5,
  0x935a, 0x91c4, 0x9032, 0x8ea4, 0x8d1b, 0x8b96, 0x8a15, 0x8898, 0x871f, 0x85ab,
  0x843a, 0x82ce, 0x8165, 0x8000
};

uint16_t mapPhaseInc(uint16_t input) {
  return (antilogTable[input & 0x3f]) >> (input >> 6);
};

// Stepped chromatic mapping
//
uint16_t midiTable[] = {
  0x11, 0x12, 0x13, 0x14, 0x16, 0x17, 0x18, 0x1a, 0x1b, 0x1d, 0x1f, 0x20, 0x22,
  0x24, 0x26, 0x29, 0x2b, 0x2e, 0x30, 0x33, 0x36, 0x3a, 0x3d, 0x41, 0x45, 0x49,
  0x4d, 0x52, 0x56, 0x5c, 0x61, 0x67, 0x6d, 0x73, 0x7a, 0x81, 0x89, 0x91, 0x9a,
  0xa3, 0xad, 0xb7, 0xc2, 0xce, 0xda, 0xe7, 0xf4, 0x103, 0x112, 0x123, 0x134,
  0x146, 0x15a, 0x16e, 0x184, 0x19b, 0x1b3, 0x1cd, 0x1e9, 0x206, 0x225, 0x245,
  0x268, 0x28c, 0x2b3, 0x2dc, 0x308, 0x336, 0x367, 0x39b, 0x3d2, 0x40c, 0x449,
  0x48b, 0x4d0, 0x519, 0x567, 0x5b9, 0x610, 0x66c, 0x6ce, 0x735, 0x7a3, 0x817,
  0x893, 0x915, 0x99f, 0xa32, 0xacd, 0xb72, 0xc20, 0xcd8, 0xd9c, 0xe6b, 0xf46,
  0x102f, 0x1125, 0x122a, 0x133f, 0x1464, 0x159a, 0x16e3, 0x183f, 0x19b1, 0x1b38,
  0x1cd6, 0x1e8d, 0x205e, 0x224b, 0x2455, 0x267e, 0x28c8, 0x2b34, 0x2dc6, 0x307f,
  0x3361, 0x366f, 0x39ac, 0x3d1a, 0x40bc, 0x4495, 0x48a9, 0x4cfc, 0x518f, 0x5669,
  0x5b8c, 0x60fe, 0x66c2
};
uint16_t mapMidi(uint16_t input) {
  return (midiTable[(1023 - input) >> 3]);
};

// Stepped Pentatonic mapping
//
uint16_t pentatonicTable[54] = {
  0, 19, 22, 26, 29, 32, 38, 43, 51, 58, 65, 77, 86, 103, 115, 129, 154, 173, 206, 231, 259, 308, 346,
  411, 461, 518, 616, 691, 822, 923, 1036, 1232, 1383, 1644, 1845, 2071, 2463, 2765, 3288,
  3691, 4143, 4927, 5530, 6577, 7382, 8286, 9854, 11060, 13153, 14764, 16572, 19708, 22121, 26306
};

uint16_t mapPentatonic(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 53);
  return (pentatonicTable[value]);
};

// Fibonacci Sequence, low-frequency
//
uint16_t fibonacciBeatTable[20] = {
  1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144
};


uint16_t mapFibonacciBeat(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 20);
  return (fibonacciBeatTable[value]);
};

// Linear, low-frequency, such as for danceable rhythms or an LFO.
//
uint16_t bpmTable[16] = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
};

uint16_t mapBPM(uint16_t input) {
  uint8_t value = (1023 - input) / (1024 / 16);
  return (bpmTable[value]);
};

void audioOn() {
#if defined(__AVR_ATmega8__)
  // ATmega8 has different registers
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  // Set up PWM to 31.25kHz, phase accurate
#endif
}


void setup() {
  pinMode(PWM_PIN, OUTPUT);
  audioOn();
  pinMode(LED_PIN, OUTPUT);
  pinMode(GATE_CONTROL, INPUT); // Use a pullup resistor to ground to clean the signal. Can't use INPUT_PULLUP because Gate is triggered with HIGH.
  /* TCB2 = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); */
  /* TCB2 = _BV(CS22); */
  /* OCR2A = 180; */
}

void loop() {
  // The loop is pretty simple - it just updates the parameters for the oscillators while the GATE pin is HIGH.
  //
  // Avoid using any functions that make extensive use of interrupts, or turn interrupts off.
  // They will cause clicks and poops in the audio.
  /*
       Remove this if() if you want it to play constantly
       OR install a switch between the GATE pin and 5v
       to switch between constant play and gate trigger.
  */

  if (digitalRead (GATE_CONTROL) == HIGH) {  // Flip to LOW if you're using active-low Gate signal.
    if (triggerGate == true) {
      grainStart();
      triggerGate = false;
    }

    grainBuild();

  } else {
    // Reset the trigger so it starts back from the beginning
    triggerGate = true;
    grainBuild ();
  }
}

void grainStart () {
  grainPhaseAcc = 0;
  grainAmp = 0x7fff;
  grain2PhaseAcc = 0;
  grain2Amp = 0x7fff;
  LED_PORT = !1 << LED_BIT; // Faster than using digitalWrite. You can use this for a Gate signal.
}

void grainBuild () {
  // Smooth frequency mapping
  //syncPhaseInc = mapPhaseInc(analogRead(SYNC_CONTROL)) / 4;

  // Stepped mapping to MIDI notes: C, Db, D, Eb, E, F...
  //syncPhaseInc = mapMidi(analogRead(SYNC_CONTROL));

  // Stepped pentatonic mapping: D, E, G, A, B
  // syncPhaseInc = mapPentatonic(analogRead(SYNC_CONTROL));

  // Stepped mapping to low-Hz Fibonacci beats: 1, 1, 2, 3, 5, 8, 13...
  // syncPhaseInc = mapFibonacciBeat(analogRead(SYNC_CONTROL));

  // Linear, low-Hz mapping: 1, 2, 3, 4...
  syncPhaseInc = mapBPM(analogRead(SYNC_CONTROL));

  // Read the grain inputs and send them to the timers
  grainPhaseInc  = mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     = analogRead(GRAIN_DECAY_CONTROL) / 8;
  grain2PhaseInc = mapPhaseInc(analogRead(GRAIN2_FREQ_CONTROL)) / 2;
  grain2Decay    = analogRead(GRAIN2_DECAY_CONTROL) / 4;
}

SIGNAL(PWM_INTERRUPT) {
  uint8_t value;
  uint16_t output;

  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    if (digitalRead (GATE_CONTROL) == HIGH) {  // Flip to LOW if you're using active-low gate signal.
      grainStart();
    }
  }

  // Increment the phase of the grain oscillators
  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  // Convert phase into a triangle wave
  value = (grainPhaseAcc >> 7) & 0xff;
  if (grainPhaseAcc & 0x8000) value = ~value;
  // Multiply by current grain amplitude to get sample
  output = value * (grainAmp >> 8);

  // Repeat for second grain
  value = (grain2PhaseAcc >> 7) & 0xff;
  if (grain2PhaseAcc & 0x8000) value = ~value;
  output += value * (grain2Amp >> 8);

  // Make the grain amplitudes decay by a factor every sample (exponential decay)
  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  // Scale output to the available range, clipping if necessary
  output >>= 9;
  if (output > 255) output = 255;

  // Output to PWM (this is faster than using analogWrite)
  PWM_VALUE = output;
  Serial.println(output);
}
