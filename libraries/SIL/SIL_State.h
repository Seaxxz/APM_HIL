#pragma once

#include <AP_HAL/AP_HAL.h>
//#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/utility/Socket.h>
#include "SIM_Multicopter.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_GPS/AP_GPS.h>
#define SIL_NUM_CHANNELS 4
class SIL_State{
public:

	SIL_State(AP_InertialSensor &ins, AP_GPS &gps) : _ins(ins),_gps(gps){}
	void read(void) ;
	bool output_ready = false;
    void _fdm_input_local(void);
    uint16_t pwm_output[SIL_NUM_CHANNELS];
    MultiCopter multicopter;
	struct Aircraft::sitl_fdm get_sitl_output();
private:
	AP_InertialSensor &_ins;
    AP_GPS &_gps;
    void _simulator_servos(Aircraft::sitl_input &input);
    float wind_speed_active = 2.0f;
    float wind_direction_active = 2.0f;
    float wind_turbulance = 1.0f;

};
