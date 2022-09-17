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
 * @file WeatherVane.hpp
 * @author Ivo Drescher
 * @author Roman Bapst <roman@auterion.com>
 *
 * Weathervane controller.
 *
 */

#pragma once

#include <px4_platform_common/module_params.h> // (OSJ) this header is needed to get the DEFINE_PARAMETERS macro
#include <matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp> // (OSJ) this is for uORB C++ s	ubscription API
#include <uORB/topics/vehicle_attitude_setpoint.h> // (OSJ) to use 'vehicle_attitude_setpoint' uORB topic in the code
#include <uORB/topics/vehicle_local_position.h> // (OSJ) to use 'vehicle_local_position' uORB topic in the code
#include <uORB/topics/vehicle_control_mode.h> // (OSJ) to use 'vehicle_control_mode' uORB topic in the code
#include <uORB/topics/vehicle_status.h> // (OSJ) to use 'vehicle_status' uORB topic in the code

class WeatherVane : public ModuleParams //(OSJ) define class "WeatherVane", inheriting ModuleParams to WeatherVane.
{
public: // (OSJ) public data member. both accessible from inside class and outside class(through object)
	WeatherVane(ModuleParams *parent); // (OSJ) Constructor of this object

	~WeatherVane() = default; // (OSJ) Destructor of this object. If this line is omitted, complier puts a default destructor.

	void setNavigatorForceDisabled(bool navigator_force_disabled) { _navigator_force_disabled = navigator_force_disabled; }; // (OSJ) define a function

	bool isActive() {return _is_active;}

	void update(); 		       //(OSJ) only defining the function, and then the actual function content is define in WeatherVane.cpp file.

	float getWeathervaneYawrate(); //(OSJ) only defining the function, and then the actual function content is define in WeatherVane.cpp file.

private: // (OSJ) private data member. only accessible inside class.
	// (OSJ) Subscriptions
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	bool _is_active{false};

	// (OSJ) local copies of status such that we don't need to copy uORB messages all the time
	bool _flag_control_manual_enabled{false};
	bool _flag_control_position_enabled{false};
	bool _navigator_force_disabled{false};

	DEFINE_PARAMETERS( // (OSJ) Use DEFINE_PARAMETERS to specify a list of parameters and their associated parameter attributes
		(ParamBool<px4::params::WV_EN>) _param_wv_en,
		(ParamFloat<px4::params::WV_ROLL_MIN>) _param_wv_roll_min,
		(ParamFloat<px4::params::WV_GAIN>) _param_wv_gain,
		(ParamFloat<px4::params::WV_YRATE_MAX>) _param_wv_yrate_max
	)
};
