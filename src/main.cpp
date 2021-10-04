#include <Arduino.h>

#define PIN_VALUE(PORT_INPUT_PIN_ADDRESS, BIT) (PORT_INPUT_PIN_ADDRESS & (1 << BIT)) >> BIT
#define INPUT_PIN(DATA_DIRECTION_REGISTER, BIT) DATA_DIRECTION_REGISTER &= ~(1 << BIT)
#define OUTPUT_PIN(DATA_DIRECTION_REGISTER, BIT) DATA_DIRECTION_REGISTER |= (1 << BIT)
#define OUTPUT_LOW(PORT_OUTPUT_PIN_ADDRESS, BIT) PORT_OUTPUT_PIN_ADDRESS &= ~(1 << BIT)

const uint8_t NUMBER_OF_BUTTONS = 3;
const uint8_t buttonPins[] = {PC0, PC1, PC2};

uint8_t previous[NUMBER_OF_BUTTONS];

const uint8_t NUMBER_OF_LEDS = 3;
const uint8_t ledPins[] = {PD6, PD3, PD5};
const char* colors[] = {"RED", "GREEN", "BLUE"};
uint8_t ledPwmValues[] = {0, 0, 0};
uint8_t resetButtonPreviousValue;

uint8_t currentValue;
uint8_t selectedLed;

volatile bool initialSelection = false;
volatile bool interrupted = false;
volatile bool selectionChanged = false;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUMBER_OF_BUTTONS; i++)
  {
    // Buttons
    INPUT_PIN(DDRC, buttonPins[i]);
    // Internal pull-up
    PORTC |= (1 << buttonPins[i]);

    // LEDs
    OUTPUT_PIN(DDRD, ledPins[i]);
    OUTPUT_LOW(PORTD, ledPins[i]);

    // Take current button pin values (hopefully they are all pulled high now (because caps))
    previous[i] = PIN_VALUE(PINC, buttonPins[i]);
  }

  // 2 Input pins for the rotary encoder.
  INPUT_PIN(DDRB, PB1);
  INPUT_PIN(DDRB, PB2);
  // Reset button input
  INPUT_PIN(DDRB, PB3);
  resetButtonPreviousValue = PIN_VALUE(PINB, PB3);

  // Interrupts
  sei();
  // Interrupts for the rotary encoder
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT3) | (1 << PCINT2);
  // Interrupts for the push buttons
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);
}

ISR(PCINT0_vect)
{
  // If at least one color LED is selected through a push button.
  if (initialSelection)
  {
    if (resetButtonPreviousValue != PIN_VALUE(PINB, PB3))
    {
      resetButtonPreviousValue = PIN_VALUE(PINB, PB3);
      
      if (resetButtonPreviousValue)
      {
        ledPwmValues[selectedLed] = 0;
      }
    }
    // Since we only interrupt on PB2 we get 1 -> 0 -> 1 and we only care about when 1
    else if (PIN_VALUE(PINB, PB2))
    {
      if (PIN_VALUE(PINB, PB1))
      {
        if (ledPwmValues[selectedLed] > 0)
        {
          ledPwmValues[selectedLed]--;
        }
      }
      else
      {
        if (ledPwmValues[selectedLed] < 255)
        {
          ledPwmValues[selectedLed]++;
        }
      }
    }

    interrupted = true;
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

      if (!currentValue) break;

      selectedLed = i;
      initialSelection = true;
      selectionChanged = true;
    }
  }
}

void loop() {
  if (!initialSelection)
  {
    return;
  }
  
  analogWrite(ledPins[selectedLed], ledPwmValues[selectedLed]);
  
  if (selectionChanged)
  {
    Serial.print("CHANGED TO "); 
    Serial.println(colors[selectedLed]);
    selectionChanged = false;
  }

  if (interrupted)
  {
    Serial.println(ledPwmValues[selectedLed]);
    interrupted = false;
  }
}
