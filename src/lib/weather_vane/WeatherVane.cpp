/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file WeatherVane.cpp
 * Weathervane controller.
 *
 */

#include "WeatherVane.hpp"
#include <mathlib/mathlib.h>

//gazebo wind /
WeatherVane::WeatherVane(ModuleParams *parent) : //(OSJ) Executing WeatherVane object constructor in WeatherVane class.
	ModuleParams(parent)
{ }

void WeatherVane::update() // (OSJ) defining update() function in WeatherVane class, declared in WeatherVane.hpp. This function does something, not returning something.
{
	vehicle_control_mode_s vehicle_control_mode;// (B) defining vehicle_control_mode_s called "vehicle_control_mode".

	// (OSJ) This part is same as void parameters_update() in the PX4 document
	if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) { /* (PX4) Tell us if there is any update to the 'vehicle_control_mode' uORB message(but not what parameter is affected)
									  (B) update variable vcm, + check flags are enabled.*/
		_flag_control_manual_enabled = vehicle_control_mode.flag_control_manual_enabled;
		_flag_control_position_enabled = vehicle_control_mode.flag_control_position_enabled;
	}

	// Weathervane needs to be enabled by parameter
	// in manual we use weathervane just if position is controlled as well
	// in mission we use weathervane except for when navigator disables it
	_is_active = _param_wv_en.get()
		     && ((_flag_control_manual_enabled && _flag_control_position_enabled)
			 || (!_flag_control_manual_enabled && !_navigator_force_disabled));
}

float WeatherVane::getWeathervaneYawrate() // (OSJ) defining getWeatherVaneYawrate() function in WeatherVane class, declared in WeatherVane.hpp
{
	// (1) body_z_sp [3d-vector, 3x1 matrix]
	// direction of desired body z axis represented in earth frame

	vehicle_attitude_setpoint_s vehicle_attitude_setpoint; // (OSJ) This two lines are related to updating parameters
	_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint);

	matrix::Vector3f body_z_sp(matrix::Quatf(vehicle_attitude_setpoint.q_d).dcm_z()); /* (OSJ) current z-axis of vehicle's attitude on xyz NED frame
											     "matrix::Quatf(vehicle_attitude_setpoint.q_d)" is four quaternions number that express vehicle's current attitude
											     	(ex1: vehicle_attitude_setpoint.q_d: [0.7021, 0.0000, 0.0000, 0.7121] (Roll: 0.0 deg, Pitch: -0.0 deg, Yaw: 90.8 deg)) (px4> listener vehicle_attitude_setpoint)
											     	(ex2: vehicle_attitude_setpoint.q_d: [0.5792, 0.0190, 0.0267, 0.8145] (Roll: 3.7 deg, Pitch: 0.0 deg, Yaw: 109.2 deg))
												(ex3: vehicle_attitude_setpoint.q_d: [0.0717, 0.0003, 0.0046, 0.9974] (Roll: 0.5 deg, Pitch: -0.0 deg, Yaw: 171.8 deg)) heading: 2.9949
											      and
											     "(vehicle_attitude_setpoint.q_d).dcm_z()" is calculation of rotation matrix using quaternions above, but only the third column is shown.
											     	(ex1: vehicle_attitude_setpoint.q_d.dcm_z() = [0, 0, 1.00003])
											     	(ex2: vehicle_attitude_setpoint.q_d.dcm_z() = [0.0618803, 0.0214847, 0.997809]) */

	// (2) R_yaw [3x3 matrix]
	/* rotate desired body z axis into new frame which is rotated in z by the current
	   heading of the vehicle. we refer to this as the heading frame. */

	vehicle_local_position_s vehicle_local_position{};
	_vehicle_local_position_sub.copy(&vehicle_local_position);

	matrix::Dcmf R_yaw = matrix::Eulerf(0.0f, 0.0f, -vehicle_local_position.heading); /* (OSJ) make a directional cosine matrix called "R_yaw". this is exactly same with rotation matrix.
											     This rotate the matrix roll: 0 rad, pitch: 0 rad, yaw: -heading rad.
											     (ex: heading: 1.5688(89deg), 2.874, ... 3.1404(179deg) (positing heading -> clockwise rotation about Z-axis, when seen from top(U in ENU). CCW when seen from bottom(D in NED)))
											     (0~3.14 rad is 0~180deg. then -3.14~0 deg, which is 180~360deg.)
											     seems like this is same as the uORB "vehicle_attitude_setpoint.yaw_body"
											     (ex: R_yaw = [0 0 -1.5688]  this value is radian. 1.5688 almost close to 90 deg)
											     Eulerf: 3x1 matrix(phi,theta,psi/(x,y,z)), Dcmf: 3x3 matrix */

	// (3)
	body_z_sp = R_yaw * body_z_sp; // (OSJ) [3x1 matrix] [no unit] (3x3 matrix) * (3x1 vector)
	body_z_sp.normalize(); // (OSJ) make it unit vector


	float roll_sp = -asinf(body_z_sp(1)); // (OSJ) - arcsin of second element of body_z_sp vector [radian]
	float roll_exceeding_treshold = 0.0f; // (OSJ) Initialize new variable called roll_exceeding_treshold to 0.0. (B)--> estimate the wind direction? //mission mode MC mode only weathervane.
	float min_roll_rad = math::radians(_param_wv_roll_min.get()); // (OSJ) _param_wv_roll_min = Minimum roll angle setpoint for weathervane controller to demand a yaw-rate.

	if (roll_sp > min_roll_rad) {
		roll_exceeding_treshold = roll_sp - min_roll_rad;

	} else if (roll_sp < -min_roll_rad) {
		roll_exceeding_treshold = roll_sp + min_roll_rad;
	}

	return math::constrain(roll_exceeding_treshold * _param_wv_gain.get(), -math::radians(_param_wv_yrate_max.get()),
			       math::radians(_param_wv_yrate_max.get())); // (OSJ) return three values: weathervane yaw rate, and max and min weathervane yaw rate[deg/s]
}
