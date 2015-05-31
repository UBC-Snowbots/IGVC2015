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
  //read sensors
  //GPS_read();
  //serial


  //motors
  hal.scheduler->delay(10);
  //debug_rcin();
  read_radio();
  //print_pwm();
  talk();
  move_pwm();
  //control motor
  //hal.console->println("test");




}

void debug_rcin() {
  uint16_t channels[8];
  hal.rcin->read(channels, 8);
  hal.console->printf_P(
  PSTR("rcin: %u %u %u %u %u %u %u %u\r\n"),
  channels[0],
  channels[1],
  channels[2],
  channels[3],
  channels[4],
  channels[5],
  channels[6],
  channels[7]);
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

void print_pwm()
{
  for (int i=0; i<8; i++) {
    hal.console->printf("ch%u: %4d ", (unsigned)i+1, (int)rc[i].control_in);
    hal.rcout->write(1, rc[i].control_in);

  }
  hal.rcout->write(0, rc_1.radio_in);
  hal.rcout->write(1, rc_2.radio_in);
  hal.console->printf("\n");
}

void move_pwm()
{
  uint16_t wheels[4]; 
  //rotate side forward
  if(rc[3].control_in < 300)
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
  /*
    hal.rcout->write(0, rc_1.radio_in);
   hal.rcout->write(1, rc_2.radio_in);*/
  /*
    hal.console->printf_P(
   PSTR("rcin: %u %u %u %u\r\n"),
   wheels[0],
   wheels[1],
   wheels[2],
   wheels[3]);
   hal.console->printf("\n");*/
}


void print_radio_values()
{
  for (int i=0; i<8; i++) {
    hal.console->printf("CH%u: %u|%u\n",
    (unsigned)i+1, 
    (unsigned)rc[i].radio_min, 
    (unsigned)rc[i].radio_max); 
  }
}


void setup_radio(void)
{	
  rc_1.radio_min = 1040;//1048,1463,1879
  rc_2.radio_min = 1040;//1054,1483,1879
  rc_3.radio_min = 1040;//1050,1466,1875
  rc_4.radio_min = 1085;
  rc_5.radio_min = 1085;
  rc_6.radio_min = 1085;
  rc_7.radio_min = 1085;
  rc_8.radio_min = 1085;

  rc_1.radio_max = 1879;
  rc_2.radio_max = 1879;
  rc_3.radio_max = 1875;
  rc_4.radio_max = 1915;
  rc_5.radio_max = 1915;
  rc_6.radio_max = 1915;
  rc_7.radio_max = 1915;
  rc_8.radio_max = 1915;

  // 3 is not trimed
  rc_1.radio_trim = 1463;//-28
  rc_2.radio_trim = 1483;//-1
  rc_3.radio_trim = 1466;//-22
  rc_4.radio_trim = 1500;
  rc_5.radio_trim = 1500;
  rc_6.radio_trim = 1500;
  rc_7.radio_trim = 1500;
  rc_8.radio_trim = 1500;

  /*
    rc_1.set_range(-250,250);
   rc_1.set_default_dead_zone(20);
   rc_2.set_range(-250,250);
   rc_2.set_default_dead_zone(20);
   rc_3.set_range(-250,250);
   rc_3.set_default_dead_zone(20);
   rc_4.set_range(-250,250);
   rc_4.set_default_dead_zone(20);*/
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
         GPS_read();
         compass_read();
          hal.console->printf("B,%f,%f,%07.2f\n",lat,lon,ToDeg(heading));
          /*
        if (count_gps==2)
        {
          compass_read();
          hal.console->printf("B,%f,%f,%07.2f\n",lat,lon,ToDeg(heading));
        }
        count_gps++;
        if(count_gps>5)
        {
          count_gps=0;
        }*/



        //accel.x.y.z,gyro.x.y.z,length
        //hal.console->println("good");


        //bytes[2]=c_x>>8;
        //bytes[3]=c_x;


        //Serial.write(bytes,24);
      }
    }
  }
}

void setup_internal_sensors()
{
  /*
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
   // we need to stop the barometer from holding the SPI bus
   hal.gpio->pinMode(40, GPIO_OUTPUT);
   hal.gpio->write(40, 1);
   #endif
   
   ins.init(AP_InertialSensor::COLD_START, 
   AP_InertialSensor::RATE_100HZ);*/

  compass.init();
  compass.set_offsets(97.39, 178.85, 217.25); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(-20));
    /*
  compass.set_offsets(-92.75, 65.57, 143.20); // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north
*/
}

void run_test()
{
  // clear out any existing samples from ins
  ins.update();
  ins.wait_for_sample(1000);

  accel = ins.get_accel();
  gyro = ins.get_gyro();
  length = accel.length();

  //cumpass
  float heading;

  compass.accumulate();
  compass.read();

  Matrix3f dcm_matrix;
  // use roll = 0, pitch = 0 for this example
  dcm_matrix.from_euler(0, 0, 0);
  heading = compass.calculate_heading(dcm_matrix);
  compass.null_offsets();

  // capture min
  const Vector3f &mag = compass.get_field();


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
        } else {
            //hal.console->println("No fix");
            lat=0.0;
            lon=0.0;
        }
        gps->new_data = false;
    }
  
  /*
  //gps->update();
  //gps->update();
  if (gps->new_data) {
    if (gps->fix) 
    {
      
      print_latlon(hal.console,gps->latitude);
       hal.console->printf(",");
       print_latlon(hal.console,gps->longitude);//B125125125
       
      hal.console->printf("%f,%f",(float)gps->latitude,(float)gps->longitude);
    }
    else
    {
      //hal.console->print("4.9262256e+08,-1.2324804e+09");
      hal.console->printf("0.0000000e+00,00.0000000e+00");
    }
    gps->new_data = false;
  }
  else
  {
    //hal.console->print("4.9262256e+08,-1.2324804e+09");
    hal.console->printf("0.0000000e+00,00.0000000e+00");
  }
  */
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



AP_HAL_MAIN();

