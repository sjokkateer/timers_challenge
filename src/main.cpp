#include <Arduino.h>

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
    DDRC &= ~(1 << buttonPins[i]);
    PORTC |= (1 << buttonPins[i]);

    // LEDs
    DDRD |= (1 << ledPins[i]);
    PORTD &= ~(1 << ledPins[i]);    

    // Take current button pin values (hopefully they are all pulled high now (because caps))
    previous[i] = (PINC & (1 << buttonPins[i])) >> buttonPins[i];
  }

  // 2 Input pins for the rotary encoder.
  DDRB &= ~(1 << PB1);
  DDRB &= ~(1 << PB2);
  // Reset button input
  DDRB &= ~(1 << PB3);
  resetButtonPreviousValue = (PINB & (1 << PB3)) >> PB3;

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
    if (resetButtonPreviousValue != (PINB & (1 << PB3)) >> PB3)
    {
      resetButtonPreviousValue = (PINB & (1 << PB3)) >> PB3;
      
      if (resetButtonPreviousValue)
      {
        ledPwmValues[selectedLed] = 0;
      }
    }
    // Since we only interrupt on PB2 we get 1 -> 0 -> 1 and we only care about when 1
    else if ((PINB & (1 << PB2)) >> PB2)
    {
      if ((PINB & (1 << PB1)) >> PB1)
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
    currentValue = (PINC & (1 << buttonPins[i])) >> buttonPins[i];

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
