#include <Arduino.h>

const uint8_t NUMBER_OF_BUTTONS = 3;
const uint8_t buttonPins[] = {PC0, PC1, PC2};

uint8_t previous[NUMBER_OF_BUTTONS];

const uint8_t NUMBER_OF_LEDS = 3;
const uint8_t ledPins[] = {PD6, PD3, PD5};

uint8_t currentValue;

void setup() {
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++)
  {
    // Buttons
    DDRC &= ~(1 << buttonPins[i]);
    PORTC |= (1 << buttonPins[i]);

    // LEDs
    DDRD |= (1 << ledPins[i]);
    PORTD &= ~(1 << ledPins[i]);

    previous[i] = (PINC & (1 << buttonPins[i])) >> buttonPins[i];
  }

  // Interrupts
  sei();
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);
}

ISR(PCINT1_vect)
{
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++)
  {
    currentValue = (PINC & (1 << buttonPins[i])) >> buttonPins[i];

    if (previous[i] != currentValue)
    {
      previous[i] = currentValue;

      if (!currentValue) break;

      PORTD ^= (1 << ledPins[i]);
    }
  }
}

void loop() {
}
