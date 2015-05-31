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
#include <AP_Compass.h>
#include <AP_BattMonitor.h>


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_InertialSensor_MPU6000 ins;
AP_Compass_HMC5843 compass;

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

Vector3f accel;
Vector3f gyro;
float length;

//gps
GPS         *gps;
AP_GPS_Auto GPS(&gps);

#define T6 1000000
#define T7 10000000

float lat;
float lon;

//led
AP_HAL::DigitalSource *a_led;
int LEDcount=0;

float heading;

AP_BattMonitor battery_mon;

int count_gps=0;
int safety_count=0;

//gps nav
float gps_y;
float gps_z;

int gps_count=0;
int gps_waypoint=0;


void setup()
{
  //hal.console->println("ArduPilot RC Channel test");
  //hal.scheduler->delay(500);
  setup_radio();
  setup_internal_sensors();
  //print_radio_values();
  GPS_setup();

  for (int i = 0; i < 30; i++) {
    read_radio();
  }

  hal.rcout->set_freq(0xFF, 490);

  hal.rcout->enable_ch(0);
  hal.rcout->enable_ch(1);
  hal.rcout->enable_ch(2);
  hal.rcout->enable_ch(3);

  hal.rcout->write(0, 1500);
  hal.rcout->write(1, 1500);
  hal.rcout->write(2, 1500);
  hal.rcout->write(3, 1500);

  a_led = hal.gpio->channel(54);//A10 output for LEDs
  a_led->mode(GPIO_OUTPUT);
  a_led->write(0);

  //battery monitor
  battery_mon.init();
  battery_mon.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
}

void loop()
{

  hal.scheduler->delay(10);
  read_radio();
  compas_nav();
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
  if(rc[3].control_in < 150)//gps
  {
    wheels[0]=1500+gps_z-gps_y;//+z*(a+b)
    wheels[1]=1500+gps_z+gps_y;
    wheels[2]=1500+gps_z+gps_y;
    wheels[3]=1500+gps_z-gps_y;

    if(LEDcount<10)
    {
      a_led->write(1);
    }
    else
    {
      a_led->write(0);
      if(LEDcount>40)
      {
        LEDcount=0;
      }
    }
    LEDcount++;
  }
  else if(rc[3].control_in < 300)
  {
    //hal.console->printf_P(PSTR("twist"));
    wheels[0]=1500+twist_z+twist_x-twist_y;//+z*(a+b)
    wheels[1]=1500+twist_z+twist_x+twist_y;
    wheels[2]=1500+twist_z-twist_x+twist_y;
    wheels[3]=1500+twist_z-twist_x-twist_y;

    if(LEDcount<20)
    {
      a_led->write(1);
    }
    else
    {
      a_led->write(0);
      if(LEDcount>40)
      {
        LEDcount=0;
      }
    }
    LEDcount++;
  }
  else if(rc[3].control_in < 650)
  {
    //hal.console->printf_P(PSTR("radio"));
    wheels[0]=1500+rc[0].control_in+rc[2].control_in-rc[1].control_in;//+z*(a+b)
    wheels[1]=1500+rc[0].control_in+rc[2].control_in+rc[1].control_in;
    wheels[2]=1500+rc[0].control_in-rc[2].control_in+rc[1].control_in;
    wheels[3]=1500+rc[0].control_in-rc[2].control_in-rc[1].control_in;

    a_led->write(1);
  }
  else
  {
    //hal.console->printf_P(PSTR("stop"));
    wheels[0]=1500;
    wheels[1]=1500;
    wheels[2]=1500;
    wheels[3]=1500;

    a_led->write(0);
  }

  //checks for battery;
  battery_mon.read();
  if(battery_mon.voltage()<9.2 || battery_mon.current_amps()>45)
  {
    safety_count++;
    if(safety_count>10)
    {

      wheels[0]=1500;
      wheels[1]=1500;
      wheels[2]=1500;
      wheels[3]=1500;
      a_led->write(0);
      //safety_count++;
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
  hal.rcout->enable_ch(2);
  hal.rcout->write(2, wheels[2]);
  hal.rcout->enable_ch(3);
  hal.rcout->write(3, wheels[3]);
}

void setup_radio(void)
{	
  rc_1.radio_min = 1040;//1048,1463,1879
  rc_2.radio_min = 1040;//1054,1483,1879
  rc_3.radio_min = 1040;//1050,1466,1875
  rc_4.radio_min = 1040;
  rc_5.radio_min = 1085;
  rc_6.radio_min = 1085;
  rc_7.radio_min = 1085;
  rc_8.radio_min = 1085;

  rc_1.radio_max = 1879;
  rc_2.radio_max = 1879;
  rc_3.radio_max = 1875;
  rc_4.radio_max = 1875;
  rc_5.radio_max = 1915;
  rc_6.radio_max = 1915;
  rc_7.radio_max = 1915;
  rc_8.radio_max = 1915;

  // 3 is not trimed
  rc_1.radio_trim = 1463;//-28
  rc_2.radio_trim = 1483;//-1
  rc_3.radio_trim = 1466;//-22
  rc_4.radio_trim = 1470;
  rc_5.radio_trim = 1500;
  rc_6.radio_trim = 1500;
  rc_7.radio_trim = 1500;
  rc_8.radio_trim = 1500;

  rc_1.set_range(-500,500);
  rc_1.set_default_dead_zone(50);
  rc_2.set_range(-500,500);
  rc_2.set_default_dead_zone(50);
  rc_3.set_range(-500,500);
  rc_3.set_default_dead_zone(50);
  rc_4.set_range(0,1000);
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
        Byte[1]= hal.console->read();
        Byte[2]= hal.console->read();
        Byte[3]= hal.console->read();
        Byte[4]= hal.console->read();
        Byte[5]= hal.console->read();
        Byte[6]= hal.console->read();
        Byte[7]= hal.console->read();
        Byte[8]= hal.console->read();
        Byte[9]= hal.console->read();
        hal.console->flush();
        //Bints[0]=100*(int)(Byte[1]-'0');
        Bints[0]=100*(int)(Byte[1]-'0')+10*(int)(Byte[2]-'0')+(int)(Byte[3]-'0');
        twist_x=Bints[0];
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
        else if(Otwist_y-twist_y>100)
        {
          twist_y=Otwist_y-100;
        }
        else if(twist_y-Otwist_y>100)
        {
          twist_y=Otwist_y+100;
        }
        else if(Otwist_z-twist_z>100)
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

        /*
      twist_x=((int)Byte[1]*8-500);
         twist_y=((int)Byte[2]*8-500);
         twist_z=((int)Byte[3]*8-500);*/


        /*
        GPS_read();
         compass_read();
         hal.console->printf("B,%f,%f,%07.2f\n",lat,lon,ToDeg(heading));
         
         */



      }
    }
  }
}

void setup_internal_sensors()
{
  compass.init();
  /*
  compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
   compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north*/
   /*
  compass.set_offsets(-92.75, 65.57, 143.20); // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0)); // set local difference between magnetic north and true north*/
  
  compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(7));
}


void GPS_setup()
{
  hal.uartB->begin(38400);
  gps = &GPS;
  gps->init(hal.uartB, GPS::GPS_ENGINE_AIRBORNE_2G);
}

void GPS_read()
{
  gps->update();
  if (gps->new_data) {
    if (gps->fix) {
      lat=(float)gps->latitude;
      lon=(float)gps->longitude;
    } 
    else {
      //hal.console->println("No fix");
      lat=0.0;
      lon=0.0;
    }
    gps->new_data = false;
  }

}

void compass_read()
{
  compass.accumulate();
  compass.read();


  Matrix3f dcm_matrix;
  // use roll = 0, pitch = 0 for this example
  dcm_matrix.from_euler(0, 0, 0);
  heading = compass.calculate_heading(dcm_matrix);
  compass.null_offsets();


  // display all to user
  //hal.console->printf("Heading: %.2f",
  //ToDeg(heading));
}

void compas_nav()
{
  compass_read();
  //gps
  GPS_read();
  //llat: 4.260734e+08, glat: 4260737., lon: -8.325089e+08, glon: -8325102., Heading: nan


  //float goal_lat=426073700;
  //float goal_lon=-832510200;

  //float goal_lat[2]={426781629,42677944};
  float goal_lat[2]={426781700,42677940};
  float goal_lon[2]={-831954900,-831954900};
  //float goal_lon[2]={-831954749,-831955031};
  
  
  float goal_lat1=426781629;
  float goal_lon1=-831954749;
  
  float goal_lat2=42677944;
  float goal_lon2=-831955031;

  float north=goal_lat[gps_waypoint]-lat;//+=north, -=south
  float west=goal_lon[gps_waypoint]-lon;// -=west, +=east

  if(north>-100 && north<100 && west >-100 && west<100)
  {
    gps_waypoint++;
  }

  //zeros exeptions
  //find desired heading
  float goal_heading=atan(west/north);
  if(north<0 && goal_heading<0)
  {
    goal_heading+=PI;
  }
  else if(north<0 && goal_heading>0)
  {
    goal_heading-=PI;
  }


  //diference in heading and desired heading
  float rotation;

  rotation=goal_heading-heading;
  if(rotation<(-PI))
  {
    rotation=(2*PI+rotation);
  }
  else if(rotation>PI)
  {
    rotation=(rotation-2*PI);
  }


  //hal.console->printf("lat: %f, glat: %f, lon: %f, glon: %f, Heading: %.2f, Real: %f, north: %f, west: %f, rotation %f\n",lat,goal_lat,lon,goal_lon,ToDeg(atan(west/north)),ToDeg(heading),north,west,rotation);
  //
  

  if(rotation>5 || rotation<(-5))
  {
    gps_y=0;
    if(rotation>0.35)
    {
      gps_z=100;
    }
    else
    {
      gps_z=-100;
    }
  }
  else if(rotation>0.4 || rotation<(-0.4))
  {
    gps_y=-100;
    if(rotation>0.2)
    {
      gps_z=75;
    }
    else
    {
      gps_z=-75;
    }
  }
  else if(rotation>0.1 || rotation<(-0.1))
  {
    gps_y=-150;
    if(rotation>0.1)
    {
      gps_z=0;
    }
    else
    {
      gps_z=0;
    }
  }
  
  if(gps_waypoint>1)
  {
    gps_y=0.0;
    gps_z=0;
  }
  
  //gps_y=0.0;
  
  //lat: 4.267817e+08, glat: 4.267794e+07, lon: -8.319549e+08, glon: -8.319550e+08,Heading: 0.00, Real: 74.57840, rotation 105.4216
  //lat: 4.267795e+08, glat: 4.267794e+07, lon: -8.319550e+08, glon: -8.319550e+08,Heading: -0.00, Real: 108.5442, rotation -108.5442


if(gps_count==10)
  {
  //hal.console->printf("lat: %f, glat: %f, lon: %f, glon: %f,Heading: %.2f, Real: %f, rotation %f\n",lat,goal_lat[gps_waypoint],lon,goal_lon[gps_waypoint],ToDeg(atan(west/north)),ToDeg(heading),ToDeg(rotation));
  }
  gps_count++;
  if(gps_count>10)
  {
    gps_count=0;
  }

  //roatate till acheved, 0 throttle, .2>20 degree,, 0.1 throtle 0.13>15, 0.2 throttle 0.1>5



}



AP_HAL_MAIN();




