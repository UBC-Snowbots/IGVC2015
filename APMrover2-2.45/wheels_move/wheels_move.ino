// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo w1;
Servo w2;
Servo w3;
Servo w4;

float b=0.665/2;
float a=0.72/2;
 
void setup() 
{ 
  w1.attach(9);
  w2.attach(10);
  w3.attach(11);
  w4.attach(8);
} 
 
 
void loop() 
{ 
  //read sensors
  //procces data
  //serial send receve
  //move car
  
  
  move(20,0,0);
  delay(3000);
  move(0,20,0);
  delay(3000);
  move(-20,0,0);
  delay(3000);
  move(0,-20,0);
  delay(3000);
  
  
} 


void move(int x, int y, int z)
{
  w1.write(y-x+z*(a+b));
  w2.write(y+x-z*(a+b));
  w3.write(y-x-z*(a+b));
  w4.write(y+x+z*(a+b));
}

