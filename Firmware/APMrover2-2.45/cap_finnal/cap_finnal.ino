#include <Servo.h>
#include <LiquidCrystal.h>

/*

 A0 = Coarse
 A1 = Fine
 
 Pin 9 = ESC
 
 D10 = on/off switch
 
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 3
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 5
 * LCD D7 pin to digital pin 6
 * LCD R/W pin to ground
 * LCD Vo pin to ground
 
 */


Servo motor;
LiquidCrystal lcd(12, 11, 3, 4, 5, 6);

void setup() {


  motor.attach(3);//inisilises servo on pin 3
  motor.write(90);//puts 0 throttle to servo
  lcd.begin(8, 2);
  lcd.print("Standby");

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
    lcd.setCursor(0, 0);
    lcd.print("On");
    lcd.setCursor(0, 1);
    lcd.print((throttle-90));
  }
  else
  {
    motor.write(90);// else put nuttral throttle
    lcd.setCursor(0, 0);
    lcd.print("Standby");
    lcd.setCursor(0, 1);
    lcd.print((throttle-90));
  }


  delay(100);        // delay in between reads for stability (0.1s)
}

