// pw-smoothRide
//
// (c) 2022 Eric Hebert
//
// Soft start for Power Wheel that is powered via a speed controller
// PWM pin SPEEDPIN is connected to the potentiometer input of the speed controller (act as a replacement pot)
// Actual pot is connected to POTPIN, and is read to compute maximum desired power level
// Accelerator pedal input is read on ACCPIN (use voltage divider to protect the microcontroller)
// Smoothes out power level from MINPOWER to desired power level at the rate of 1% per RAMPT ms.
//
// TO DO:
// - Implement usable serial debugging
// - Add fire
// - (done)Debounce accelerator pedal (wind down pwm so that it doesn't restart at MINPOWER, with time decay)
// - (done) Show current power level on LED's instead of desired power level when acc()
// - bug: does not seem to start at MINPOWER anymore
// Runs on Arduino Duemilanove
//

#include <Arduino.h>
#include <FastLED.h>

//#define DEBUG_SERIAL // Enable serial debugging

// IO Pin definitions
#define SPEEDPIN 3 // Pin where the MOSFETs are conected
#define ACCPIN 4   // Pin that reads Power Wheel accelerator input
#define POTPIN A0  // Pin that reads potentiometer input

// Ramp parameters
#define MINPOWER 10 // minimum power level to start ramping up from (%)
#define MINPWMPOWER 23 // PWM power (0-255) at 0% powerLevel. Makes sure that the potientiometer range excludes a zone where there isn't enough voltage to push the vehicle forward.
#define RAMPT 25   // time (ms) per percent of power ramp up. lower value = faster acceleration
#define DECAYT 10 // time(ms) per percent of power ramp down

// FastLED

#define LED_PIN 7
#define COLOR_ORDER GRB
#define CHIPSET WS2812B
#define NUM_LEDS 14

#define BRIGHTNESS 127

bool gReverseDirection = false;

CRGB leds[NUM_LEDS];

// Global Variables
uint8_t pwmPowerPercent = 0;

// Function prototypes

void rampUp(uint8_t);
bool acc();
uint8_t powerLevel(uint8_t);

// Function definitions

void setup()
{

#ifdef DEBUG_SERIAL
  Serial.begin(9600);
#endif

  // Pin setups
  pinMode(SPEEDPIN, OUTPUT);
  pinMode(ACCPIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // FastLED setup
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

void loop()
{

  static unsigned long int timestamp = 0;
  if (millis() - timestamp < 100)
  {
    // Show powerLevel on leds
    // if acc(), show current pwm output level. Else, show desired power level.
    static uint8_t d_powerLevel = 0;
    if (acc())
    {
      d_powerLevel = map(pwmPowerPercent, 0, 100, 0, NUM_LEDS);
    }
    else if (!acc()) {
      d_powerLevel = map(analogRead(POTPIN), 0, 1023, 0, NUM_LEDS);
    }
    
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[(NUM_LEDS - 1) - i] = CRGB::Black; // reset to black in case it was previously set to a color
      if (i < d_powerLevel)
      {
        if (i < 8) leds[(NUM_LEDS - 1) - i] = CRGB::Green; // LEDS are reversed
        else if (i < 11) leds[(NUM_LEDS - 1) - i] = CRGB::Yellow;
        else leds[(NUM_LEDS - 1) - i] = CRGB::Red;
      }
    }

    FastLED.show();
    timestamp = millis();
  }

  rampUp(map(analogRead(POTPIN), 0, 1023, 0, 100));
}

// Converts 0-100% to 8 bit PWM value for analogWrite function
uint8_t powerLevel(uint8_t powerPercent)
{
  return map(powerPercent, 0, 100, MINPWMPOWER, 255);
}

// Returns true if Accelerator pedal is pressed
bool acc()
{
  return digitalRead(ACCPIN);
}

// Ramps up power level as per parameters
void rampUp(uint8_t maxPower)
{
  static uint8_t powerPercent = MINPOWER; // initially start at MINPOWER
  static bool lastAcc = acc();            // Remember past accelerator state
  if (acc())                              // Only run this if accelerator pedal is pressed
  {
    if (!lastAcc) // Run this if accelerator was off before this function was run this time
    {
      if (powerPercent < MINPOWER) powerPercent = MINPOWER;
      lastAcc = true; // make sure we don't reset to MINPOWER next time
    }

    digitalWrite(LED_BUILTIN, HIGH); // Turn on built-in LED to indicate that accelerator is pressed.

    static unsigned long int timestamp = 0;
    if (millis() - timestamp > RAMPT) // Increase by 1 percent every x ms
    {
      if (powerPercent > maxPower) // Check if we are over first, as maxPower could be < MINPOWER
      {
        powerPercent = maxPower; // Stop increasing at maxPower
      }
      analogWrite(SPEEDPIN, powerLevel(powerPercent));
      pwmPowerPercent = powerPercent;
      powerPercent++; // Increase power by 1 percent for next loop

      timestamp = millis();
    }
  }
  if (!acc())
  {
    static unsigned long int timestamp = 0;
    if (millis() - timestamp > DECAYT)
    {
      if (powerPercent > 0) powerPercent--;
      timestamp = millis();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off built-in LED
    //powerPercent = MINPOWER;        // Reset power to maximum desired power when accelerator pedal is depressed.
    
    analogWrite(SPEEDPIN, powerLevel(maxPower)); // set output to maxPower in case accelerator input is disabled (cut wire?)
  }
}
