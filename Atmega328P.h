#ifndef Atmega328P_H
#define Atmega328P_H
//made by Rick Overhorst
//an implementation of the Microcontroller interface
//includes
#include <stdint.h>
#include <avr/io.h>
#include <Arduino.h>
#include <math.h>
#include "functions.h"
#include "Microcontroller.h"

#define DEBUG 0
#define clockspeed 16000000
//#include<std.h>
class Atmega328P : public Microcontroller
{
    private:

    public:
       //UART
       void    UARTInit(uint8_t dataBits, uint8_t parityBit, uint8_t stopBits, float baud, uint8_t speed, uint8_t RT);  //Function to initialize the UART, takes the amount of data bits (5,6,6,8 or 9 bits), no, even or an odd parity bit, the amound of stop bits (1 or 2), baudrate and whether or not receive and transmit should be enabled
       void    UARTBegin(long int baud);                                                                                //Function to begin the UART with the defaul settings and a baudrate.
       void    UARTSend(uint16_t data);                                                                                 //Function to send 9 bits, warning: not yet implemented
       void    UARTSend(uint8_t data);                                                                                  //function to send less 9 bits
       void    UARTSendBytes(uint8_t bytes[], uint8_t amountOfBytes);                                                   //Function to send multiple bytes. Takes an Array of bytes to send and the amount of bytes to send
       uint8_t UARTRead();                                                                                              //Function to read a byte
       void    UARTREADBytes(uint8_t *bytes, uint8_t amountOfBytes);                                                    //Function to read multiple bytes. Takes an Array of bytes/pointer to put the bytes in when they are read and the amount of bytes to read
       void    println(uint8_t *bytes);                                                                                 //Function to sends a char array/string and sets  a new line
       void    print(uint8_t *bytes);                                                                                   //Function to sends a char array/string and doesn't set a new line

       //GPIO
       void    setDPM(uint8_t pin, uint8_t value);  //Stands for set Digital Pin Mode. Sets the data directions of a digital pin (pinnumbers are based on the arduino uno board) value = 0 or 1 for input or output respectively
       void    setAPM(uint8_t pin, uint8_t value);  //Stands for set AnaLog Pin Mode. Sets the data directions of a analog pin (pinnumbers are based on the arduino uno board) value = 0 or 1 for input or output respectively
       void    DW(uint8_t pin, uint8_t value);      //Stands for Digital Write(dw). Sets the pin (based on the pinnumbers on the arduino uno board) to a logical LOW or HIGH based on value which can be 0 or 1.
       uint8_t DR(uint8_t pin);                     //Stands for Digital Read, if the pin has been set to input this function can read if it's set to a logical LOW or HIGH
       void    AW(uint8_t pin, uint8_t value);      //This function is not PWM, this function mimics the DW (Digital Write) but with the analog pins
       uint16_t AR(uint8_t pin);                    //Stands for Analog Read. Function to the read an analog value value on the pin (based on the Arduino UNO pinnumbers)
       void    toggleDP(uint8_t pin);               //Stands for toggle Digital Pin, if the pin (pinnumbers are based on the arduino uno board) is a logical LOW then it returns 0 if it's HIGH then the function returns 1
       uint8_t getPinStat(uint8_t pin);             //When a digital pin is set to output this function can get wheter the pin is set to a logical LOW or HIGH
       void    binToLed(uint8_t byte);              //Function to display a byte on 8 leds in binary, PORTB is used for this.

       //I2C
       void    sendI2CStart();            //Function to send a start signal on the I2C bus
       void    sendI2CReStart();          //Function to send a repeated start signal on the I2C bus
       void    sendI2CStop();             //Function to send a stop signal on the I2C bus
       void    sendI2CAddr(uint8_t addr); //Function to send a slave address on the I2C bus
       void    sendI2CData(uint8_t data); //Function to send data on the I2C bus
       uint8_t readI2CData();             //Function to read a byte from the the I2C bus

       //EEPROM
       void    EEPROM_Write(uint32_t addr, uint8_t data);                                 //Function to write to the EEPROM. addr is the address to wich the data is written
       void    EEPROM_WriteBytes(uint32_t addr, uint8_t data[], uint32_t amountOfBytes);  //Function to write multiple bytes to EEPROM. The function takes an address (uint32_t addr) of the first memory address, an array with data bytes and the amount of bytes.
       uint8_t EEPROM_Read(uint32_t addr);                                                //Function to read from the EEPROM. addr is the address from wich the data is read
       void    EEPROM_ReadBytes(uint32_t addr, uint8_t data[], uint32_t amountOfBytes);   //Function to read multiple bytes from the EEPROM. The function takes


       //misc
       //gets the bit for the data (a byte) at the index
       uint8_t getBit(uint8_t data, uint8_t index);
  };
#endif
