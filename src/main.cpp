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
// - check if maxPower < MINPOWER (see TO DO comment)
// - Add fire

#include <Arduino.h>

//#define DEBUG_SERIAL // Enable serial debugging

// IO Pin definitions
#define SPEEDPIN 3 // Pin where the speed controller is conected to
#define ACCPIN 4   // Pin that reads Power Wheel accelerator input
#define POTPIN A0  // Pin that reads potentiometer input

// Ramp parameters
#define MINPOWER 25 // minimum power level to start ramping up from
#define RAMPT 25    // time (ms) per percent of power ramp up. lower value = faster acceleration

// Function prototypes

void rampUp(uint8_t);
bool acc();
uint8_t powerLevel(uint8_t);

// Function definitions

// Converts 0-100% to 8 bit PWM value for analogWrite function
uint8_t powerLevel(uint8_t powerPercent)
{
  return map(powerPercent, 0, 100, 25, 229);
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
      powerPercent = MINPOWER;
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
      powerPercent++; // Increase power by 1 percent for next loop

      timestamp = millis();
    }
  }
  if (!acc())
  {
    digitalWrite(LED_BUILTIN, LOW); // turn off built-in LED
    powerPercent = MINPOWER;        // Reset power to minimum when accelerator pedal is depressed. TO DO: check if maxPower < MINPOWER
    // Don't write powerPercent to output pin, so that the value set on the pot can be read on the display.
    analogWrite(SPEEDPIN, powerLevel(maxPower)); // set output to maxPower so value can be read on the display.
  }
}

void setup()
{

#ifdef DEBUG_SERIAL
  Serial.begin(9600);
#endif

  // Pin setups
  pinMode(SPEEDPIN, OUTPUT);
  pinMode(ACCPIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
#ifdef DEBUG_SERIAL
  static unsigned long int timestamp = 0;
  if (millis() - timestamp < 500)
  {
    Serial.println(map(analogRead(POTPIN), 0, 1023, 0, 100));
    timestamp = millis();
  }
#endif

  rampUp(map(analogRead(POTPIN), 0, 1023, 0, 100));
}