// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_InertialSensor driver.
//

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>

#include <AP_Math.h>    // ArduPilot Mega Vector/Matrix math Library
#include <AP_Declination.h>
#include <AP_Compass.h> // Compass Library
#include <ros.h>
#include <std_msgs/String.h>


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_InertialSensor_MPU6000 ins;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
AP_Compass_PX4 compass;
#else
AP_Compass_HMC5843 compass;
#endif
uint32_t timer;

void setup(void)
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

    hal.scheduler->delay(1000);
    timer = hal.scheduler->micros();
}


void loop(void)
{
    int16_t user_input;

    hal.console->println();
    hal.console->println_P(PSTR(
    "\nCompass Heading (Degrees)    read_time ms     Accelerometer(x,y,z) m/s^2     Accel magnitude     Gyroscope (x,y,z) rad/s"));

    // wait for user input

run_test();
    
}

void run_test()
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


        hal.console->println();
    


        // wait until we have a sample
        ins.wait_for_sample(read_time);

        // read samples from ins
        ins.update();
        accel = ins.get_accel();
        gyro = ins.get_gyro();

        length = accel.length();

hal.console->printf_P(PSTR("%.2f\t\t\t\t%u \t\t  %4.2f  %4.2f  %4.2f \t \t %4.2f \t\t\t%4.2f %4.2f %4.2f\n"), 
								ToDeg(heading), (unsigned)read_time, accel.x, accel.y, accel.z, length, gyro.x, gyro.y, gyro.z);






}
}

AP_HAL_MAIN();
