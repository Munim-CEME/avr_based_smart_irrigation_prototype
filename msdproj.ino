#include <avr/io.h>
#define moistureSensorPin A0  // Analog pin for the moisture sensor
#define pwmPin 9              // Digital pin for PWM output

void setup() {
  OutputPin(pwmPin);   // Set the PWM pin as an output
  Serial.begin(9600);        // Initialize serial communication
}

void loop() {
  int sensorValue = analogVal(moistureSensorPin);  // Read the sensor value
  int moisturePercentage = mapValue(sensorValue, 0, 1023, 100, 0);  // Convert to percentage
  
  Serial.print("Moisture Percentage: ");
  Serial.print(moisturePercentage);
  Serial.println("%");
  
  if (moisturePercentage < 20) {
    PWM(pwmPin, 255);   // Set PWM signal to 255
  } else if (moisturePercentage >= 20 && moisturePercentage <= 70) {
    PWM(pwmPin, 150);   // Set PWM signal to 150
  } else {
    PWM(pwmPin, 10);    // Set PWM signal to 10
  }
  
  delay(1000);  // Wait for 1 second before taking the next reading
}



void PWM(uint8_t pin, int value) {
  // Check if the pin supports PWM
  if (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11) {
    // Set the corresponding Timer/Counter for the pin
    switch (pin) {
      case 3:
        TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
        TCCR2B |= _BV(CS21);
        OCR2B = value;
        break;
      case 5:
        TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
        TCCR0B |= _BV(CS01);
        OCR0B = value;
        break;
      case 6:
        TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01);
        TCCR0B |= _BV(CS01);
        OCR0A = value;
        break;
      case 9:
        TCCR1A |= _BV(COM1A1) | _BV(WGM10);
        TCCR1B |= _BV(CS11) | _BV(WGM12);
        OCR1A = value;
        break;
      case 10:
        TCCR1A |= _BV(COM1B1) | _BV(WGM10);
        TCCR1B |= _BV(CS11) | _BV(WGM12);
        OCR1B = value;
        break;
      case 11:
        TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21);
        TCCR2B |= _BV(CS21);
        OCR2A = value;
        break;
    }
  } else if (pin == 13) {
    // Onboard LED (special case)
    if (value > 127) {
      PORTB |= _BV(PORTB5);
    } else {
      PORTB &= ~_BV(PORTB5);
    }
  }
}



int analogVal(uint8_t pin) {
  // Set the reference voltage to AVCC
  ADMUX = (1 << REFS0);
  
  // Set the ADC channel to read from the specified pin
  ADMUX |= pin & 0x07;
  
  // Enable ADC and start the conversion
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  
  // Wait for the conversion to complete
  while (ADCSRA & (1 << ADSC));
  
  // Read and return the converted value
  return ADC;
}



void OutputPin(uint8_t pin) {
  // Check if the pin is within the valid range
  if (pin < 0 || pin > 19)
    return;
  
  // Calculate the port and bit values for the pin
  uint8_t port = pin / 8;
  uint8_t bit = pin % 8;
  
  // Set or clear the DDR register bit based on the specified mode
  
    switch (port) {
      case 0:
        DDRD |= (1 << bit);
        break;
      case 1:
        DDRC |= (1 << bit);
        break;
      case 2:
        DDRB |= (1 << bit);
        break;
    }
   
}

long mapValue(long value, long fromLow, long fromHigh, long toLow, long toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}



