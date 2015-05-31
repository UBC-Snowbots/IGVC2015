
#include <Servo.h> 

Servo myservo; 

int pos=90;

const int analogInPin = A0;

int starValue=0;
int sensorValue = 0;

void setup() 
{ 
  Serial.begin(9600); 
  myservo.attach(9);
  myservo.write(pos);
  starValue = analogRead(analogInPin); 
  sensorValue=starValue;
  delay(1000);
} 

void loop() 
{ 
  sensorValue= analogRead(analogInPin);
  pos=(sensorValue-starValue)/5+90;

  Serial.print("pos: ");      
  Serial.print(pos);
  Serial.print(", setting: ");      
  Serial.println((sensorValue-starValue)/5);  
  myservo.write(pos);
  delay(100);  
} 

