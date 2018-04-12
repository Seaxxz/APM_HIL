
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/select.h>
#include "SIL_State.h"
#include <../ArduCopter/Copter.h>
extern const AP_HAL::HAL& hal;

struct Aircraft::sitl_fdm SIL_State::get_sitl_output()
{
	return multicopter.sitl_output;
}

void SIL_State::read()
{
    output_ready = true;
    for(int ch=0;ch<4;ch++)
    {
    	pwm_output[ch] = hal.rcout->read(ch);
    }
}
/*
  get FDM input from a local model
 */
void SIL_State::_fdm_input_local(void)
{
    Aircraft::sitl_input input;
    read();
    // construct servos structure for FDM
    _simulator_servos(input);

    // update the model
    multicopter.update(input);

    _ins.set_accel(0,multicopter.sitl_output.angAccel);

    Vector3f gyro_out;
    gyro_out.x=(float)(multicopter.sitl_output.rollRate);
    gyro_out.y=(float)(multicopter.sitl_output.pitchRate);
    gyro_out.z=(float)(multicopter.sitl_output.yawRate);
    _ins.set_gyro(0,gyro_out);
}

/*
  create sitl_input structure for sending to FDM
 */
void SIL_State::_simulator_servos(Aircraft::sitl_input &input)
{
    static uint32_t last_update_usec;

    /* this maps the registers used for PWM outputs. The RC
     * driver updates these whenever it wants the channel output
     * to change */
    uint8_t i;

    if (last_update_usec == 0 || !output_ready) {
        for (i=0; i<SIL_NUM_CHANNELS; i++) {
            pwm_output[i] = 1000;
        }
    }

    // output at chosen framerate
    uint32_t now = AP_HAL::micros();
    last_update_usec = now;

    // pass wind into simulators, using a wind gradient below 60m
    float altitude = 2;
    float wind_speed = 0;
    float wind_direction = 0;
        // The EKF does not like step inputs so this LPF keeps it happy.
    wind_speed = wind_speed_active = (0.95f*wind_speed_active) + (0.05f*wind_speed);
    wind_direction = wind_direction_active = (0.95f*wind_direction_active) + (0.05f*wind_direction);

    if (altitude < 0) {
        altitude = 0;
    }
    if (altitude < 60) {
        wind_speed *= sqrtf(MAX(altitude / 60, 0));
    }

    input.wind.speed = wind_speed;
    input.wind.direction = wind_direction;
    input.wind.turbulence = wind_turbulance?wind_turbulance:0;

    for (i=0; i<SIL_NUM_CHANNELS; i++) {
            input.servos[i] = pwm_output[i];
    }

        // run checks on each motor
        for (i=0; i<4; i++) {
            // check motors do not exceed their limits
            if (input.servos[i] > 2000) input.servos[i] = 2000;
            if (input.servos[i] < 1000) input.servos[i] = 1000;
    }


}

