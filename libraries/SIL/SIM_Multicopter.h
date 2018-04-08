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

#pragma once

#include "Aircraft.h"
#include "SIM_Motor.h"
#include "SIM_Frame.h"


/*
  a multicopter simulator
 */
class MultiCopter {
public:
    void init(void);
    /* update model by one time step */
    void update(const struct Aircraft::sitl_input &input);
	struct Aircraft::sitl_fdm sitl_output;
	Aircraft aircraft;

protected:

    // calculate rotational and linear accelerations
    void calculate_forces(const struct Aircraft::sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel);
    Frame *frame;


};

