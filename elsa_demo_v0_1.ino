//mechanum main
//uses MPU6000,RCoutput,RCinput, blizzard4

//**********WARNING*only works with diydrones apm IDE and modified battery monitor libary*************//
//download AP_BatteryMon.h from electrical projects on google drive
//if this does not compile check those two things first

//used for radio receiver
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

#include <AP_BatteryMon.h>//note needs the modified library to function
#include <AP_Compass.h> // Compass Library

#ifdef DOES_ARDUINO_NOT_SUPPORT_CUSTOM_INCLUDE_DIRECTORIES
#include <AP_ADC_AnalogSource.h>
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;//required for all APM code

RC_Channel rc_1(CH_1);
RC_Channel rc_2(CH_2);
RC_Channel rc_3(CH_3);
RC_Channel rc_4(CH_4);
RC_Channel rc_5(CH_5);
RC_Channel rc_6(CH_6);
RC_Channel rc_7(CH_7);
RC_Channel rc_8(CH_8);
RC_Channel *rc = &rc_1;

int twist_y=0;//throttle
int twist_z=0;//rotation

int Otwist_y=0;//old throttle
int Otwist_z=0;//old rotation

long leftE,rightE; //encoder ticks and velocity variables
long OleftE, OrightE;
int velocity_count=0;
float L_speed, R_speed;
long Lspeed, Rspeed;

int compdeg;//compass heading in degrees

float left_motor_cal, right_motor_cal =1; //calibration variables
float voltage1, voltage2 = 0;
float batt_mon1_vol, batt_mon2_vol = 0;
int voltage_count = 0;

AP_BatteryMon battery_mon1(1,0);//default pins
AP_BatteryMon battery_mon2(2,3);//TODO select actual pins to use for second battery monitor

int safety_count=0;//used for when battery voltage is low
int healthy_count=0;

AP_HAL::DigitalSource *a_led;//pins for safety LED
AP_HAL::DigitalSource *b_led;

//initializing the compass
AP_InertialSensor_MPU6000 ins;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4//used to set up up the imu sensors
AP_Compass_PX4 compass;
#else
AP_Compass_HMC5843 compass;
#endif
uint32_t timer;


#define Encoder  0x09  //encoder i2c address

void setup()
{
  //setup receiver
  setup_radio();
  
  for (int i = 0; i < 30; i++) {
    read_radio();
  }

  hal.rcout->set_freq(0xFF, 490);

  hal.rcout->enable_ch(0);
  hal.rcout->enable_ch(1);

  hal.rcout->write(0, 1500);//write neutral throttle to esc
  hal.rcout->write(1, 1500);

  //battery monitor
  battery_mon1.init();
  battery_mon1.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
  battery_mon2.init();
  battery_mon2.set_monitoring(AP_BATT_MONITOR_VOLTAGE_AND_CURRENT);
  //LED control
  a_led = hal.gpio->channel(54);//A10 output for LEDs
  a_led->mode(GPIO_OUTPUT);
  a_led->write(0);
  b_led = hal.gpio->channel(53);//A9 output for LEDs
  b_led->mode(GPIO_OUTPUT);
  b_led->write(0);
  
  //set up compass and MPU6000
  setup_compass();
  
  //set up encoders
  hal.i2c->writeRegister(Encoder,0x00,0x00);
}

void loop()
{
  hal.scheduler->delay(10);
  read_radio();
  talk();//send and receive serial messages
  move_pwm();
  run_compass();
  velocity();
  motor_calibration();
}

void read_radio()//reads the pwm input for rc receiver
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

void move_pwm()// commands the esc
{
  uint16_t wheels[2]; 
  //rotate side forward
  if(rc[2].control_in < 300)//manual control
  {
    //hal.console->printf_P(PSTR("twist"));
    wheels[0]=1500+twist_z-twist_y;//+z*(a+b)
    wheels[1]=1500+twist_z+twist_y;
    a_led->write(0);//LED solid
    b_led->write(1);
  }
  else if(rc[2].control_in < 650)//autonomous
  {
    //hal.console->printf_P(PSTR("radio"));
    wheels[0]=1500+rc[3].control_in+rc[1].control_in;//+z*(a+b)
    wheels[1]=1500+rc[3].control_in-rc[1].control_in;
    a_led->write(1);//LED Blinking
    b_led->write(1);
  }
  else//wireless e-stop
  {
    //hal.console->printf_P(PSTR("stop"));
    wheels[0]=1500;
    wheels[1]=1500;
    a_led->write(0);//LED off
    b_led->write(0);
  }
  
  //TODO: Add in compass set-up and read functions (can be "copied") from Mecanum 2

  //checks for battery;
  battery_mon1.read();
  batt_mon1_vol = battery_mon1.voltage();
  batt_mon2_vol = battery_mon2.voltage();
  
  if(batt_mon1_vol<9.5 || battery_mon1.current_amps()>19 || batt_mon2_vol<9.5 || battery_mon2.current_amps()>19)
  {
    safety_count++;
    if(safety_count>10)
    {
      wheels[0]=1500;
      wheels[1]=1500;
      
      a_led->write(0);
      
      //2 Hz blink
      
      if (safety_count%100 < 50)
      	b_led->write(0);

      else
      	b_led->write(1);
      
    }
  }
  else if(safety_count>0)
  {
    safety_count--;
    if(safety_count>20)
    {
    	healthy_count++;
    	if (healthy_count > 10)
    	{
    		safety_count = 0;
    		healthy_count = 0;
    	}
    }
  }
  hal.rcout->enable_ch(0);
  hal.rcout->write(0, wheels[0]);
  hal.rcout->enable_ch(1);
  hal.rcout->write(1, wheels[1]);
}

void setup_radio(void)
{	
  rc_1.radio_min = 1050;//sets up the minimum value from reciver
  rc_2.radio_min = 1076;
  rc_3.radio_min = 1051;
  rc_4.radio_min = 1055;
  rc_5.radio_min = 1085;
  rc_6.radio_min = 1085;
  rc_7.radio_min = 1085;
  rc_8.radio_min = 1085;
  
  rc_1.radio_max = 1888;//setup maximum value from reciver
  rc_2.radio_max = 1893;
  rc_3.radio_max = 1883;
  rc_4.radio_max = 1886;
  rc_5.radio_max = 1915;
  rc_6.radio_max = 1915;
  rc_7.radio_max = 1915;
  rc_8.radio_max = 1915;

  // 3 is not trimed
  rc_1.radio_trim = 1472;//setup netral value
  rc_2.radio_trim = 1496;
  rc_3.radio_trim = 1500;
  rc_4.radio_trim = 1471;
  rc_5.radio_trim = 1553;
  rc_6.radio_trim = 1499;
  rc_7.radio_trim = 1498;
  rc_8.radio_trim = 1500;

  rc_1.set_range(-500,500);//set the range the revicer values are converted to
  rc_1.set_default_dead_zone(50);
  rc_2.set_range(-500,500);
  rc_2.set_default_dead_zone(50);
  rc_3.set_range(0,1000);//this is different as it is the one used for the e-stop
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
  char Byte[7];// used to recive values from serial
  int Bints[3];//used when chars is bitshifted into ints
  uint8_t bytes[20];//used to send into
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
        //TODO: send compass information to the laptop
        //TODO: send Rspeed, Lspeed
        char outbytes[10];
        outbytes[0]=compdeg>>8;
        outbytes[1]=compdeg;
        outbytes[2]=Rspeed>>24;
        outbytes[3]=Rspeed>>16;
        outbytes[4]=Rspeed>>8;
        outbytes[5]=Rspeed;
        outbytes[6]=Lspeed>>24;
        outbytes[7]=Lspeed>>16;
        outbytes[8]=Lspeed>>8;
        outbytes[9]=Lspeed;
        
        hal.console->printf("%c", outbytes[10]);
      }
    }
  }
}

void setup_compass()
{
  compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
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
        hal.console->println("compass initialisation failed!");
        while (1) ;
    }

    compass.set_offsets(0,0,0); // set offsets to account for surrounding interference
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

    // loop as long as user does not press a key
    while( !hal.console->available() ) {

      

 timer = hal.scheduler->micros();
        compass.read();
        unsigned long read_time = hal.scheduler->micros() - timer;
        float heading;

        if (!compass.healthy()) {
            hal.console->println("not healthy");
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
}

void motor_calibration(){
  
  //double check that 1 = left, 2 = right
  
  if(voltage_count<500)
  {
  voltage1 += batt_mon1_vol;
  voltage2 += batt_mon2_vol;
  
  voltage_count++;
  }
  
  else
  {
  voltage1 = voltage1/(500*12.1);
  voltage2 = voltage2/(500*12.1);
  
  left_motor_cal = voltage1; //multiply by left hardware constant
  right_motor_cal = voltage2; //multiply by right hardware constant
  
  voltage1 = 0;
  voltage2 = 0;
  voltage_count = 0;
  }
  
  //TODO: Find hardware constant (if any)
  //TODO: Insert the right_motor_cal/left_motor_cal where necessary (multiplies what where)
}  

AP_HAL_MAIN();
