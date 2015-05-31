// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 90;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(pos); 
 pinMode(10, INPUT); 
} 
 
 
void loop() 
{ 
  
   bool winch = digitalRead(10);
   if(winch)
  {
    myservo.write(103);              // tell servo to go to position in variable 'pos' 
     
  }
  else
  {
    myservo.write(90);
  }
  delay(50);

} 
