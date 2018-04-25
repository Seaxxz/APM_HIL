/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  multicopter simulator class
*/

#include "SIM_Multicopter.h"
//#include <AP_Motors/AP_Motors.h>
#include <stdio.h>
#include <../ArduCopter/Copter.h>

void MultiCopter::init()
{
	static int num_init=1;
	if(num_init==1)
	{
		num_init++;
	    frame = Frame::find_frame();
		aircraft.mass = 1.5f;
	    aircraft.frame_height = 0.1;
	    aircraft.home1.alt = 451 * 100;
	    aircraft.home1.lat = 30.876216 * 1e7;
	    aircraft.home1.lng = 104.161364 * 1e7;
		aircraft.dcm.from_euler(0.0f, 0.0f, 0.0f);
	}
	frame->init(aircraft.mass, 0.51, 15, 4*radians(360));//质量 悬停油门 最终速度 最终旋转速度
}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct Aircraft::sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(aircraft, input, rot_accel, body_accel);
}
    
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct Aircraft::sitl_input &input)
{
	init();
    // get wind vector setup
	aircraft.update_wind(input);

    Vector3f rot_accel;

    calculate_forces(input, rot_accel, aircraft.accel_body);

    aircraft.update_dynamics(rot_accel);

    // update lat/lon/altitude
    aircraft.update_position();

    aircraft.time_advance();

    aircraft.fill_fdm(sitl_output);

}
