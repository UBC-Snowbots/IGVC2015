//mechanum main
//uses MPU6000,RCoutput,RCinput, blizzard4


#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Declination.h>

#include <AP_BattMonitor.h>

#ifdef DOES_ARDUINO_NOT_SUPPORT_CUSTOM_INCLUDE_DIRECTORIES
#include <AP_ADC_AnalogSource.h>
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

RC_Channel rc_1(CH_1);
RC_Channel rc_2(CH_2);
RC_Channel rc_3(CH_3);
RC_Channel rc_4(CH_4);
RC_Channel rc_5(CH_5);
RC_Channel rc_6(CH_6);
RC_Channel rc_7(CH_7);
RC_Channel rc_8(CH_8);
RC_Channel *rc = &rc_1;

int twist_x=0;
int twist_y=0;
int twist_z=0;

int Otwist_x=0;
int Otwist_y=0;
int Otwist_z=0;

AP_BattMonitor battery_mon;

int safety_count=0;

void setup()
{
  setup_radio();
  
  for (int i = 0; i < 30; i++) {
    read_radio();
  }

  hal.rcout->set_freq(0xFF, 490);

  hal.rcout->enable_ch(0);
  hal.rcout->enable_ch(1);

  hal.rcout->write(0, 1500);
  hal.rcout->write(1, 1500);

  //battery monitor
  battery_mon.init();
  battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
}

void loop()
{
  hal.scheduler->delay(10);
  read_radio();
  talk();
  move_pwm();
}

void read_radio()
{
  rc_1.set_pwm(hal.rcin->read(CH_1));
  rc_2.set_pwm(hal.rcin->read(CH_2));
  rc_3.set_pwm(hal.rcin->read(CH_3));
  rc_4.set_pwm(hal.rcin->read(CH_4));
  rc_5.set_pwm(hal.rcin->read(CH_5));
  rc_6.set_pwm(hal.rcin->read(CH_6));
  rc_7.set_pwm(hal.rcin->read(CH_7));
  rc_8.set_pwm(hal.rcin->read(CH_8));
}

void move_pwm()
{
  uint16_t wheels[4]; 
  //rotate side forward
  if(rc[2].control_in < 300)
  {
    //hal.console->printf_P(PSTR("twist"));
    wheels[0]=1500+twist_z-twist_y;//+z*(a+b)
    wheels[1]=1500+twist_z+twist_y;
  }
  else if(rc[2].control_in < 650)
  {
    //hal.console->printf_P(PSTR("radio"));
    wheels[0]=1500+rc[3].control_in+rc[1].control_in;//+z*(a+b)
    wheels[1]=1500+rc[3].control_in-rc[1].control_in;
  }
  else
  {
    //hal.console->printf_P(PSTR("stop"));
    wheels[0]=1500;
    wheels[1]=1500;
  }
  
  //TODO: add in function to grab rotation of wheels and time information to calculate velocity 
  // this function will be called by talk
  

  //checks for battery;
  battery_mon.read();
  //TODO: Add in check for second battery monitor
  //TODO: (nice to have) add in LED blinking pattern when battery is too low
  //TODO: check that the current is ok? 
  if(battery_mon.voltage()<9 || battery_mon.current_amps()>45)
  {
    safety_count++;
    if(safety_count>10)
    {
      wheels[0]=1500;
      wheels[1]=1500;
    }
  }
  else if(safety_count>0)
  {
    safety_count--;
  }
  hal.rcout->enable_ch(0);
  hal.rcout->write(0, wheels[0]);
  hal.rcout->enable_ch(1);
  hal.rcout->write(1, wheels[1]);
}

void setup_radio(void)
{	
  rc_1.radio_min = 1050;
  rc_2.radio_min = 1076;
  rc_3.radio_min = 1051;
  rc_4.radio_min = 1055;
  rc_5.radio_min = 1085;
  rc_6.radio_min = 1085;
  rc_7.radio_min = 1085;
  rc_8.radio_min = 1085;
  
  rc_1.radio_max = 1888;
  rc_2.radio_max = 1893;
  rc_3.radio_max = 1883;
  rc_4.radio_max = 1886;
  rc_5.radio_max = 1915;
  rc_6.radio_max = 1915;
  rc_7.radio_max = 1915;
  rc_8.radio_max = 1915;

  // 3 is not trimed
  rc_1.radio_trim = 1472;
  rc_2.radio_trim = 1496;
  rc_3.radio_trim = 1500;
  rc_4.radio_trim = 1471;
  rc_5.radio_trim = 1553;
  rc_6.radio_trim = 1499;
  rc_7.radio_trim = 1498;
  rc_8.radio_trim = 1500;

  rc_1.set_range(-500,500);
  rc_1.set_default_dead_zone(50);
  rc_2.set_range(-500,500);
  rc_2.set_default_dead_zone(50);
  rc_3.set_range(0,1000);
  rc_3.set_default_dead_zone(50);
  rc_4.set_range(-500,500);
  rc_4.set_default_dead_zone(50);

  rc_5.set_range(1000,2000);
  rc_5.set_default_dead_zone(20);
  rc_6.set_range(1000,2000);
  rc_6.set_default_dead_zone(20);
  rc_7.set_range(1000,2000);
  rc_7.set_default_dead_zone(20);
  rc_8.set_range(1000,2000);
  rc_8.set_default_dead_zone(20);

  return;
}

void talk()
{
  //uint8_t Byte[6];
  char Byte[10];
  int Bints[3];
  uint8_t bytes[20];
  unsigned test=0;

  if(hal.console->available() >=10)
  { 
    //hal.console->println("talked");
    while(hal.console->available() >=10)
    {
      Byte[0]= hal.console->read();
      //hal.console->println(Byte[0]);
      if(Byte[0]=='B')// make sure all data begains with a zero
      {
        Byte[4]= hal.console->read();//swaped to try and fix wrong incorect input
        Byte[5]= hal.console->read();
        Byte[6]= hal.console->read();
        Byte[1]= hal.console->read();
        Byte[2]= hal.console->read();
        Byte[3]= hal.console->read();
        Byte[7]= hal.console->read();
        Byte[8]= hal.console->read();
        Byte[9]= hal.console->read();
        hal.console->flush();
        //Bints[0]=100*(int)(Byte[1]-'0');
        Bints[0]=100*(int)(Byte[1]-'0')+10*(int)(Byte[2]-'0')+(int)(Byte[3]-'0');
        //twist_x=Bints[0];
        Bints[1]=100*(int)(Byte[4]-'0')+10*(int)(Byte[5]-'0')+(int)(Byte[6]-'0');
        Bints[2]=100*(int)(Byte[7]-'0')+10*(int)(Byte[8]-'0')+(int)(Byte[9]-'0');
        twist_x=4*Bints[0]-500;
        twist_y=4*Bints[1]-500;
        twist_z=4*Bints[2]-500;

        if(Otwist_x-twist_x>100)
        {
          twist_x=Otwist_x-100;
        }
        else if(twist_x-Otwist_x>100)
        {
          twist_x=Otwist_x+100;
        }
        
        if(Otwist_y-twist_y>100)
        {
          twist_y=Otwist_y-100;
        }
        else if(twist_y-Otwist_y>100)
        {
          twist_y=Otwist_y+100;
        }
        if(Otwist_z-twist_z>100)
        {
          twist_z=Otwist_z-100;
        }
        else if(twist_z-Otwist_z>100)
        {
          twist_z=Otwist_z+100;
        }
        Otwist_x=twist_x;
        Otwist_y=twist_y;
        Otwist_z=twist_z;
        //TODO: send compass information to the laptop
        //TODO: send velocity information to the laptop
      }
    }
  }
}

AP_HAL_MAIN();
