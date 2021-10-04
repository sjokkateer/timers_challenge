#include <Arduino.h>

// Input pin related defs
#define PIN_VALUE(PORT_INPUT_PIN_ADDRESS, BIT) (PORT_INPUT_PIN_ADDRESS & (1 << BIT)) >> BIT
#define INPUT_PIN(DATA_DIRECTION_REGISTER, BIT) DATA_DIRECTION_REGISTER &= ~(1 << BIT)

// Output pin related defs
#define OUTPUT_PIN(DDRX, BIT) DDRX |= (1 << BIT)
#define OUTPUT_LOW(DDRX, BIT) DDRX &= ~(1 << BIT)
#define OUTPUT_HIGH(DDRX, BIT) DDRX |= (1 << BIT)

// Taking care of 0 compare values for each color
#define HANDLE_RED (OCR0A == BOTTOM) ? OUTPUT_LOW(DDRD, PD6) : OUTPUT_HIGH(DDRD, PD6)
#define HANDLE_GREEN (OCR2A == BOTTOM) ? OUTPUT_LOW(DDRB, PB3) : OUTPUT_HIGH(DDRB, PB3)
#define HANDLE_BLUE (OCR0B == BOTTOM) ? OUTPUT_LOW(DDRD, PD5) : OUTPUT_HIGH(DDRD, PD5)

// Alias the registers that control color for readability
#define RED OCR0A
#define GREEN OCR2A
#define BLUE OCR0B

// 'consts'
#define TOP 255
#define BOTTOM 0

// R, G, B
const uint8_t NUMBER_OF_LEDS = 3;
const uint8_t LED_PINS[] = {PD6, PB3, PD5};
volatile uint8_t *ledPwmValues[] = {&RED, &GREEN, &BLUE};

const uint8_t NUMBER_OF_BUTTONS = 3;
const uint8_t buttonPins[] = {PC0, PC1, PC2};

uint8_t previous[NUMBER_OF_BUTTONS];

uint8_t resetButtonPreviousValue;

uint8_t currentValue;
uint8_t selectedLed;

volatile bool initialSelection = false;

void setup()
{
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++)
  {
    // Buttons
    INPUT_PIN(DDRC, buttonPins[i]);
    // Internal pull-up
    PORTC |= (1 << buttonPins[i]);
  }

  // 2 Input pins for the rotary encoder.
  INPUT_PIN(DDRB, PB1);
  INPUT_PIN(DDRB, PB2);
  // Reset button input
  INPUT_PIN(DDRB, PB4);
  resetButtonPreviousValue = PIN_VALUE(PINB, PB4);

  // Set all previous values to determine which button will be pressed in ISR
  for (uint8_t i; i < NUMBER_OF_BUTTONS; i++)
  {
    previous[i] = PIN_VALUE(PINC, buttonPins[i]);
  }

  // SET ALL OUTPUT PINS
  // Set clear on compare match for OC0[A-B], set at bottom (non-inverting)
  TCCR0A = 0b10100011;
  // Set clear on compare match for OC2A, set at bottom (non-inverting)
  TCCR2A = 0b10000011;

  // Red
  OUTPUT_PIN(DDRD, PD6);
  // Green
  OUTPUT_PIN(DDRB, PB3);
  // Blue
  OUTPUT_PIN(DDRD, PD5);

  RED = BOTTOM;
  GREEN = BOTTOM;
  BLUE = BOTTOM;

  // If set to 0 we just set the pin low to have no mixing of colors
  // it also seems that setting the pin back high would make it use
  // the compare value and PWM again.
  HANDLE_RED;
  HANDLE_GREEN;
  HANDLE_BLUE;

  // Interrupts
  sei();
  // Interrupts for the rotary encoder
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT4) | (1 << PCINT2);
  // Interrupts for the push buttons
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);

  // Reset timers
  TCNT0 = BOTTOM;
  TCNT2 = BOTTOM;
}

ISR(PCINT0_vect)
{
  // If at least one color LED is selected through a push button.
  if (initialSelection)
  {
    if (resetButtonPreviousValue != PIN_VALUE(PINB, PB4))
    {
      resetButtonPreviousValue = PIN_VALUE(PINB, PB4);

      if (resetButtonPreviousValue)
      {
        *ledPwmValues[selectedLed] = BOTTOM;
      }

      HANDLE_RED;
      HANDLE_GREEN;
      HANDLE_BLUE;
    }
    // Since we only interrupt on PB2 we get 1 -> 0 -> 1 and we only care about when 1
    else if (PIN_VALUE(PINB, PB2))
    {
      if (PIN_VALUE(PINB, PB1))
      {
        if (*ledPwmValues[selectedLed] > BOTTOM)
        {
          *ledPwmValues[selectedLed] -= 1;
        }
      }
      else
      {
        if (*ledPwmValues[selectedLed] < TOP)
        {
          *ledPwmValues[selectedLed] += 1;
        }
      }

      HANDLE_RED;
      HANDLE_GREEN;
      HANDLE_BLUE;
    }
  }
}

ISR(PCINT1_vect)
{
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++)
  {
    currentValue = PIN_VALUE(PINC, buttonPins[i]);

    if (previous[i] != currentValue)
    {
      previous[i] = currentValue;

      // It went from high to low, so break and wait for
      // next change.
      if (!currentValue)
        break;

      // Set flag for rotary encoder ISR to execute procedure
      initialSelection = true;
      selectedLed = i;
    }
  }
}

void loop()
{
}
