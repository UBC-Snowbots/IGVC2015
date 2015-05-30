// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>


int Rout=1245;
int Lout=1123;

void setup()
{
  Wire.begin(2);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event
}

void loop()
{
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
  String Rstr=String(Rout);
  String Lstr=String(Lout);
  String output=Rstr;
  output+=Lstr;
  char carray[9];
  
  output.toCharArray(carray, 9);
  
  Wire.write(carray); // respond with message of 6 bytes
  //Wire.print(Lout);                    // as expected by master
}
