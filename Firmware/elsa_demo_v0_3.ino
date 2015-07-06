//Competition version of Elsa
//Simplified version
//No battery monitor

//used for radio receiver
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3

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
#include <AP_Compass.h> // Compass Library

#ifdef DOES_ARDUINO_NOT_SUPPORT_CUSTOM_INCLUDE_DIRECTORIES
#include <AP_ADC_AnalogSource.h>
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;//required for all APM code

RC_Channel rc_1(CH_1);
RC_Channel rc_2(CH_2);
RC_Channel rc_3(CH_3);
RC_Channel rc_4(CH_4);
RC_Channel *rc = &rc_1;

int twist_y=0;//throttle
int twist_z=0;//rotation

int Otwist_y=0;//old throttle
int Otwist_z=0;//old rotation

long Lspeed = 0;
long Rspeed = 0;

int compdeg;//compass heading in degrees

float left_motor_cal = 1;
float right_motor_cal = 1; //calibration variables

AP_HAL::DigitalSource *a_led;//pins for safety LED
int LEDcount=0;

//initializing the compass
AP_InertialSensor_MPU6000 ins;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4//used to set up up the imu sensors
AP_Compass_PX4 compass;
#else
AP_Compass_HMC5843 compass;
#endif
uint32_t timer;

void setup()
{
  //setup receiver
  setup_radio();
  
  for (int i = 0; i < 30; i++) {
    read_radio();
  }

  hal.rcout->set_freq(0xFF, 490);

  hal.rcout->enable_ch(0);
  hal.rcout->enable_ch(3);

  hal.rcout->write(0, 1500);//write neutral throttle to esc
  hal.rcout->write(3, 1500);

  //LED control
    a_led = hal.gpio->channel(54); //A0 pin
    a_led->mode(GPIO_OUTPUT);
    a_led->write(0);
  
    setup_compass(); //MUST COMMENT OUT IF COMPASS IS NOT PLUGGED IN
  
}

void loop()
{
  hal.scheduler->delay(10);
  read_radio();
  talk();//send and receive serial messages
  move_pwm();
  run_compass(); //MUST COMMENT OUT IF COMPASS IS NOT PLUGGED IN
}

void read_radio()//reads the pwm input for rc receiver
{
  rc_1.set_pwm(hal.rcin->read(CH_1));
  rc_2.set_pwm(hal.rcin->read(CH_2));
  rc_3.set_pwm(hal.rcin->read(CH_3));
  rc_4.set_pwm(hal.rcin->read(CH_4));
}

void move_pwm()// commands the esc
{
  uint16_t wheels[2]; 
  
  if(rc[2].control_in < 300) //wireless estop
  {
    a_led->write(0);
    wheels[0]=1500;
    wheels[1]=1500;
    
  }
  else if(rc[2].control_in < 650) //manual control
  {
    wheels[0]=1500+(rc[3].control_in-rc[1].control_in)*right_motor_cal;
    wheels[1]=1500+(rc[3].control_in+rc[1].control_in)*left_motor_cal+30;
    a_led->write(1);
  }
  
  else //autonomous mode
  {
    wheels[0]=1500+(twist_z-twist_y)*right_motor_cal;
    wheels[1]=1500+(twist_z+twist_y)*left_motor_cal;
        
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
  
  hal.rcout->enable_ch(0);
  hal.rcout->write(0, wheels[0]); //right wheel - output pin 1
  hal.rcout->enable_ch(3);
  hal.rcout->write(3, wheels[1]); //left wheel - output pin 4
}

void setup_radio(void)
{	
  rc_1.radio_min = 1055;//sets up the minimum value from reciver
  rc_2.radio_min = 1056;
  rc_3.radio_min = 1077;
  rc_4.radio_min = 1028;
  
  rc_1.radio_max = 1890;//setup maximum value from reciver
  rc_2.radio_max = 1892;
  rc_3.radio_max = 1906;
  rc_4.radio_max = 1858;

  // 3 is not trimed
  rc_1.radio_trim = 1470;//setup netral value
  rc_2.radio_trim = 1476;
  rc_3.radio_trim = 1472;
  rc_4.radio_trim = 1446;

  rc_1.set_range(-500,500);//set the range the revicer values are converted to
  rc_1.set_default_dead_zone(50);
  rc_2.set_range(-500,500);
  rc_2.set_default_dead_zone(50);
  rc_3.set_range(0,1000);//this is different as it is the one used for the e-stop
  rc_3.set_default_dead_zone(50);
  rc_4.set_range(-500,500);
  rc_4.set_default_dead_zone(50);

  return;
}

void talk()
{
  //uint8_t Byte[6];
  char Byte[7];// used to recive values from serial
  int Bints[3];//used when chars is bitshifted into ints
  uint8_t bytes[20];//used to send into
  unsigned test=0;

  if(hal.console->available() >=7)
  { 
    //hal.console->println("talked");
    while(hal.console->available() >=7)
    {
      Byte[0]= hal.console->read();
      ///hal.console->println(Byte[0]);
      if(Byte[0]=='B')// make sure all data begains with a zero
      {
        Byte[1]= hal.console->read();
        Byte[2]= hal.console->read();
        Byte[3]= hal.console->read();
        Byte[4]= hal.console->read();
        Byte[5]= hal.console->read();
        Byte[6]= hal.console->read();
        hal.console->flush();
        Bints[0]=100*(int)(Byte[1]-'0')+10*(int)(Byte[2]-'0')+(int)(Byte[3]-'0');
        Bints[1]=100*(int)(Byte[4]-'0')+10*(int)(Byte[5]-'0')+(int)(Byte[6]-'0');
        twist_y=4*Bints[0]-500;
        twist_z=4*Bints[1]-500;

        /*if(Otwist_x-twist_x>100)// limits rapidthrottle value changes
          twist_x=Otwist_x-100;
        else if(twist_x-Otwist_x>100)
          twist_x=Otwist_x+100;*/
        
        if(Otwist_y-twist_y>100)// limits rapidthrottle value changes
          twist_y=Otwist_y-100;
        else if(twist_y-Otwist_y>100)
          twist_y=Otwist_y+100;
          
        if(Otwist_z-twist_z>100)
          twist_z=Otwist_z-100;
        else if(twist_z-Otwist_z>100)
          twist_z=Otwist_z+100;
          
        //Otwist_x=twist_x;
        Otwist_y=twist_y;
        Otwist_z=twist_z;
        
        hal.console->printf("%04d,%07d,%07d.\n", compdeg, Rspeed, Lspeed);
        hal.scheduler->delay(2);
      }
    }
  }
}

void setup_compass()
{
  compass.set_offsets(-116.10, -12.79, 188.20); // set offsets to account for surrounding interference
  compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

  hal.scheduler->delay(1000);
  timer = hal.scheduler->micros();

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ);

    if (!compass.init()) {

//hal.console->println("compass initialisation failed!");
        while (1) ;
    }

    compass.set_offsets(-116.10, -12.79, 188.20); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north
}

void run_compass()//compass function, remove prints and console reads
{
    Vector3f accel;
    Vector3f gyro;
    float length;
	uint8_t counter = 0;
    static float min[3], max[3], offset[3];

    compass.accumulate();

    // flush any user input
    while( hal.console->available() ) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();


      

 timer = hal.scheduler->micros();
        compass.read();
        unsigned long read_time = hal.scheduler->micros() - timer;
        float heading;

        if (!compass.healthy()) {
            //hal.console->println("not healthy");
            return;
        }
	Matrix3f dcm_matrix;
	// use roll = 0, pitch = 0 for this example
	dcm_matrix.from_euler(0, 0, 0);
        heading = compass.calculate_heading(dcm_matrix);
        compass.null_offsets();

        // capture min
        const Vector3f &mag = compass.get_field();
        if( mag.x < min[0] )
            min[0] = mag.x;
        if( mag.y < min[1] )
            min[1] = mag.y;
        if( mag.z < min[2] )
            min[2] = mag.z;

        // capture max
        if( mag.x > max[0] )
            max[0] = mag.x;
        if( mag.y > max[1] )
            max[1] = mag.y;
        if( mag.z > max[2] )
            max[2] = mag.z;

        // calculate offsets
        offset[0] = -(max[0]+min[0])/2;
        offset[1] = -(max[1]+min[1])/2;
        offset[2] = -(max[2]+min[2])/2;

        // display all to user


        //hal.console->println();
    


        // wait until we have a sample
        ins.wait_for_sample(read_time);

        // read samples from ins
        ins.update();
        accel = ins.get_accel();
        gyro = ins.get_gyro();

        length = accel.length();

	compdeg=ToDeg(heading);
}

AP_HAL_MAIN();
