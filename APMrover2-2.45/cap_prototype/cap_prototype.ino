#include <Servo.h> 
 
Servo myservo; 
 
int pos = 90; 

char inByte = 0;
 
void setup() 
{ 
  Serial.begin(9600);
  
  myservo.attach(9);
  myservo.write(pos);
  delay(1000);
} 

void loop() 
{ 
  if (Serial.available() > 0) {
  inByte = Serial.read();
  
  if(inByte=='0')
  pos=0;
  else if(inByte=='+')
  pos+=5;
  else if(inByte=='-')
  pos-=5;
  else if(inByte=='>')
  pos+=1;
  else if(inByte=='<')
  pos-=1;
  
  Serial.print("pos : ");
  Serial.println(pos);
  }
                                   // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);  
} 
