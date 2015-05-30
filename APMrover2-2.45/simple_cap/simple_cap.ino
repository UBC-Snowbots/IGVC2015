#include <Servo.h>
/*

 A0 = Coarse
 A1 = Fine
 
 Pin 9 = ESC
 
 D10 = on/off switch
 
 */


Servo motor;

void setup() {


  motor.attach(3);//inisilises servo on pin 3
  motor.write(90);//puts 0 throttle to servo

  pinMode(10, INPUT);// inisalize the pin the go/no go switch is connected to 
}

void loop() {

  int course = analogRead(A0);//course gain pot
  int fine = analogRead(A1);//fine gain pot
  bool winch = digitalRead(10);
  int throttle = 85+course/51+fine/102;
  if (throttle<90)
  {
    throttle=90;
  }

  if(winch)
  {
    motor.write(throttle);//send throttle value to esc if winch is switch is pressed
  }
  else
  {
    motor.write(90);// else put nuttral throttle
  }


  delay(100);        // delay in between reads for stability (0.1s)
}

