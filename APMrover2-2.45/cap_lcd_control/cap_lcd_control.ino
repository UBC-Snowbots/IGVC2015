#include <Servo.h>

/*
A0 = coarse
A1 = fine

pin 9 = esc

*/

Servo motor;

void setup() {
  
  
  motor.attach(3);//inisilises servo on pin 3
  motor.write(90);//puts 0 throttle to servo
  
  Serial.begin(9600);//buad rate of LCD is 9600
  delay(500);//LCD takes 0.5 seconds to start up
  
  pinMode(4, INPUT);// inisalize the pin the go/no go switch is connected to
  
}



void loop() {
  
  int course = analogRead(A0);//course gain pot
  int fine = analogRead(A1);//fine gain pot
  bool winch = digitalRead(4);
  // print out the value you read:
  int inthrottle = course*10+fine;
  int throttle = inthrottle/128+90;// calculates throttle value for motor
  
  Serial.write(254); // move cursor to beginning of first line
  Serial.write(128);
  Serial.write("throttle: ");
  Serial.print(inthrottle, DEC);// displays input value on LCD
  Serial.write(254); // move cursor to beginning of first line
  Serial.write(192);
  Serial.write("throttle: ");
  Serial.print(inthrottle, DEC);// displays input value on LCD
  
  if(winch)
  {
    motor.write(inthrottle);//send throttle value to esc if winch is switch is pressed
    Serial.write(254); // move cursor to beginning of first line
    Serial.write(192);
    Serial.write("outputing");
  }
  else
  {
    motor.write(90);// else put nuttral throttle
    Serial.write(254); // move cursor to beginning of first line
    Serial.write(192);
    Serial.write("stopped");
  }
  
  
  delay(100);        // delay in between reads for stability (0.1s)
}
