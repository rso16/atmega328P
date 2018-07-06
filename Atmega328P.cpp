#include "Atmega328P.h"
//Made by Rick Overhorst


//UART
//Function to initialize the UART, takes the amount of data bits (5,6,6,8 or 9 bits), no, even or an odd parity bit, the amound of stop bits (1 or 2), baudrate and whether or not receive and transmit should be enabled
void Atmega328P::UARTInit(uint8_t dataBits, uint8_t parityBit, uint8_t stopBits, float baud, uint8_t speed, uint8_t RT)
{
  uint16_t ubrr = round(((clockspeed / ((speed + 1) * 8 * baud)) - 1));
  UCSR0A &= ~(1 << U2X0);
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  // Enable receiver and transmitter
  UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<UCSZ02);
  /* Set frame format: 8data, 1 stop bit */
  UCSR0C = (3<<UCSZ00);
}

//Function to begin the UART with the defaul settings and a baudrate.
void Atmega328P::UARTBegin(long int baud)
{
  UARTInit(0, 0, 0, baud, NORMALSPEED, 0);
}

//Function to send 9 bits, warning: not yet implemented
void Atmega328P::UARTSend(uint16_t data)
{
  //not yet implemented
}

//function to send less 9 bits
void Atmega328P::UARTSend(uint8_t data)
{
  //From the Atmega328P datasheet
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) );
  /* Put data into buffer, sends the data */
  UDR0 = data;
 }

//Function to send multiple bytes. Takes an Array of bytes to send and the amount of bytes to send
void Atmega328P::UARTSendBytes(uint8_t bytes[], uint8_t amountOfBytes)
{
  int index = 0;
  while(index < amountOfBytes - 1)
  {
    UARTSend(bytes[index]);
    index++;
  }
}

//Function to read a byte
uint8_t Atmega328P::UARTRead()
{
  int counter = 0;
  while ((!(UCSR0A & _BV(RXC0))) && counter < 75)
  {
    _delay_ms(1);
    counter++;
  }
  return (uint8_t) UDR0;
}

//Function to read multiple bytes. Takes an Array of bytes/pointer to put the bytes in when they are read and the amount of bytes to read
void Atmega328P::UARTREADBytes(uint8_t *bytes, uint8_t amountOfBytes)
{
  int index = 0;
  uint8_t* temp = bytes;
  temp = bytes + (sizeof(uint8_t));
    while(index <= amountOfBytes)
    {
      temp = bytes + (sizeof(uint8_t) * index);
      *temp =  UARTRead();
      index++;
    }
}

//Function to sends a char array/string and sets  a new line
void Atmega328P::print(uint8_t *bytes)
{
  int index = 0;
  while (bytes[index] != '\0')
  {
    index++;
  }
  UARTSendBytes(bytes, index + 1 );
}

//Function to sends a char array/string and doesn't set a new line
void Atmega328P::println(uint8_t *bytes)
{
  print(bytes);
  UARTSend((uint8_t) 0x0A);
  UARTSend((uint8_t) 0x0D);
}

//GPIO

//Stands for set Digital Pin Mode. Sets the data directions of a digital pin (pinnumbers are based on the arduino uno board) value = 0 or 1 for input or output respectively
void Atmega328P::setDPM(uint8_t pin, uint8_t value)
{
  if(value==0)
  {
    if(pin >=0 && pin<=7)
    {
      DDRD&= ~(1 << pin);
    }
    else if(pin>=8 && pin<=13)
    {
      DDRB&= ~(1 << (pin-8));
    }
  }
  else if(value==1)
  {
    if(pin >=0 && pin<=7)
    {
      DDRD|=1<<pin;
    }
    else if(pin>=8 && pin<=13)
    {
      DDRB|=1<<(pin-8);
    }
  }
}

//Stands for set AnaLog Pin Mode. Sets the data directions of a analog pin (pinnumbers are based on the arduino uno board) value = 0 or 1 for input or output respectively
void Atmega328P::setAPM(uint8_t pin, uint8_t value)
{
  if(value==0)
  {
      DDRC&= ~(1 << pin);
  }
  else if(value==1)
  {
      DDRC|=1<<pin;
  }
}

//Stands for Digital Write(dw). Sets the pin (based on the pinnumbers on the arduino uno board) to a logical LOW or HIGH based on value which can be 0 or 1.
void Atmega328P::DW(uint8_t pin, uint8_t value)
{
  if(value==0)
  {
    if(pin >=0 && pin<=7)
    {
      PORTD&= ~(1 << pin);
    }
    else if(pin>=8 && pin<=13)
    {
      PORTB&= ~(1 << (pin-8));
    }
  }
  else if(value==1)
  {
    if(pin >=0 && pin<=7)
    {
        PORTD|=1<<pin;
    }
    else if(pin>=8 && pin<=13)
    {
      PORTB|=1<<(pin-8);
    }
  }
}

//Stands for Digital Read, if the pin has been set to input this function can read if it's set to a logical LOW or HIGH
uint8_t Atmega328P::DR(uint8_t pin)
{
    uint8_t bit=0B0;
    if(pin >=0 && pin<=7)
    {
      bit = getBit(PIND,pin);
    }
    else if(pin>=8 && pin<=13)
    {
      bit = getBit(PINB,pin-8);
    }
    return bit;
}

//This function is not PWM, this function mimics the DW (Digital Write) but with the analog pins
void Atmega328P::AW(uint8_t pin, uint8_t value)
{
  if(value==0)
  {
    PORTC&= ~(1 << pin);
  }
  else if(value==1)
  {
    PORTC|=1<<pin;
  }
}

//Stands for Analog Read. Function to the read an analog value value on the pin (based on the Arduino UNO pinnumbers)
uint16_t Atmega328P::AR(uint8_t pin)
{
  ADMUX &= 0xf0;
  ADMUX |= pin;
  ADMUX |= (1<<REFS0);
  ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
  ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC));
  return ADC;
}

//Stands for toggle Digital Pin, if the pin (pinnumbers are based on the arduino uno board) is a logical LOW then it returns 0 if it's HIGH then the function returns 1
void Atmega328P::toggleDP(uint8_t pin)
{
  uint8_t value = getPinStat(pin);
  DW(pin,!value);
}

//When a digital pin is set to output this function can get wheter the pin is set to a logical LOW or HIGH
uint8_t Atmega328P::getPinStat(uint8_t pin)
{
  uint8_t bit=0B0;
  if(pin >=0 && pin<=7)
  {
    bit = getBit(PORTD,pin);
  }
  else if(pin>=8 && pin<=13)
  {
    bit = getBit(PORTB,pin-8);
  }
  return bit;
}

//Function to display a byte on 8 leds in binary, PORTB is used for this.
void Atmega328P::binToLed(uint8_t byte)
{
    int index = 7;
    int pin = 13;
    while(index >= 0)
    {
      DW(pin,getBit(byte, index));
      index--;
      pin--;
    }
}

//I2C

//Function to send a start signal on the I2C bus
void Atmega328P::sendI2CStart()
{
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while (!(TWCR &(1<<TWINT)));
  if(DEBUG){Serial.println(TWSR,HEX);};
}

//Function to send a repeated start signal on the I2C bus
void Atmega328P::sendI2CReStart()
{
  TWCR = (1<<TWSTA)|(1<<TWINT);
  TWCR&=~(1<<TWSTO);
  // while (!(TWCR &(1<<TWINT)));
  if(DEBUG){Serial.println(TWSR,HEX);};
}

//Function to send a stop signal on the I2C bus
void Atmega328P::sendI2CStop()
{
  TWCR = (1<<TWINT) |(1<<TWSTA);
  _delay_ms(10);
}

//Function to send a slave address on the I2C bus
void Atmega328P::sendI2CAddr(uint8_t addr)
{
  TWDR = addr;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR &(1<<TWINT)));
  if(DEBUG){Serial.println(TWSR,HEX);};
}

//Function to send data on the I2C bus
void Atmega328P::sendI2CData(uint8_t data)
{
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while (!(TWCR &  (1<<TWINT)));
  if(DEBUG){Serial.println(TWSR,HEX);};
}

//Function to read a byte from the the I2C bus
uint8_t Atmega328P::readI2CData()
{
  while (!(TWCR &  (1<<TWINT)));
  uint8_t data = TWDR;
  return data;
}


//Function to write to the EEPROM. addr is the address to wich the data is written
void Atmega328P::EEPROM_Write(uint32_t addr, uint8_t data)
{
  /* Wait for completion of previous write */
  while (EECR & (1<<EEPE));
  /* Set up address and Data Registers */
  EEAR = addr;
  EEDR = data;
  /* Write logical one to EEMPE */
  EECR |= (1<<EEMPE);
  /* Start eeprom write by setting EEPE */
  EECR |= (1<<EEPE);
}

//Function to write multiple bytes to EEPROM. The function takes an address (uint32_t addr) of the first memory address, an array with data bytes and the amount of bytes.
void Atmega328P::EEPROM_WriteBytes(uint32_t addr, uint8_t data[], uint32_t amountOfBytes)
{
  for (size_t i = 0; i < amountOfBytes; i++)
  {
    EEPROM_Write(addr + i, data[i]);
  }
}

//Function to read from the EEPROM. addr is the address from wich the data is read
uint8_t Atmega328P::EEPROM_Read(uint32_t addr)
{
  /* Wait for completion of previous write */
  while (EECR & (1<<EEPE));
  /* Set up address register */
  EEAR = addr;
  /* Start eeprom read by writing EERE */
  EECR |= (1<<EERE);
  /* Return data from Data Register */
  return EEDR;
}

//Function to read multiple bytes from the EEPROM. The function takes
void Atmega328P::EEPROM_ReadBytes(uint32_t addr, uint8_t data[], uint32_t amountOfBytes)
{
  for (size_t i = 0; i < amountOfBytes; i++)
  {
    data[i] = EEPROM_Read(addr + i);
  }
}

//MISC

//gets the bit for the data (a byte) at the index
uint8_t Atmega328P::getBit(uint8_t data, uint8_t index)
{
  //data HEX number from which the bit is asked
  //index is from lsb to msb ie. 0100 is 2 (3210)
  uint8_t bit = 0B0;
  data &= 1 << index;
  bit = data >> index;
  return bit;
}
