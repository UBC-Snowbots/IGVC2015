
#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


#define Encoder  0x09  //encoder i2c address

long leftE,rightE; //encoder ticks and velocity variables
long OleftE, OrightE;
int velocity_count=0;
float L_speed, R_speed;
long Lspeed, Rspeed;

void setup() {
	hal.console->println_P(PSTR("Empty setup"));
//set up encoders
  hal.i2c->writeRegister(Encoder,0x00,0x00);
  //hal.console->printf_P(PSTR("done setup"));
}
void loop() {
  hal.scheduler->delay(100);
velocity();
}

void velocity()
{

  //only updates speed every 10 loops = 10 Hz
  if (velocity_count<10)
  {
    velocity_count++;
  }
  else
  {
  read_Encoder();
  
  R_speed = (rightE-OrightE)*651.944/0.1;
  L_speed = (leftE-OleftE)*651.944/0.1;
  Rspeed = R_speed*1000;
  Lspeed = L_speed*1000;
  OleftE = leftE;
  OrightE = rightE;
  velocity_count = 0;
  }
}


void read_Encoder()//talks to the encoder MCU via i2c
{
	uint8_t data[6];
	uint8_t stat = hal.i2c->readRegisters(Encoder,0x01,8, data);
	if (stat == 0){
        
        leftE = data[0] << 24;
        leftE |= data[1] << 16;
        leftE |= data[2] << 8;
        leftE |= data[3];
        
        rightE = data[4] << 24;
        rightE |= data[5] << 16;
        rightE |= data[6] << 8;
        rightE |= data[7];
	}
  hal.console->print(leftE);
  hal.console->print_P(PSTR(", "));
  hal.console->println(rightE);
}

AP_HAL_MAIN();
