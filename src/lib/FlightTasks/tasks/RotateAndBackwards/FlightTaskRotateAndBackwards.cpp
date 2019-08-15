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
 * @file FlightTaskRotateAndBackwards.cpp
 */

#include "FlightTaskRotateAndBackwards.hpp"

bool FlightTaskRotateAndBackwards::activate()
{
	bool ret = FlightTask:activate();

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);

	_origin_z = _position(2);

	_yawspeed_setpoint = 45.0f * 3.142f / 180.f;
	_velocity_setpoint(2) =  -1.0f;

	return ret;
}

bool FlightTaskRotateAndBackwards::update()
{
	float diff_z = _position(2) - _origin_z;

	if (diff_z <= -8.0f) {
		_velocity_setpoint(2) = 1.0f;
		_yawspeed_setpoint = 45.0f * 3.142f / 180.f * -1.0f;
	} else if (diff_z >= 0.0f) {
		_velocity_setpoint(2) = -1.0f;
		_yawspeed_setpoint = 45.0f * 3.142f / 180.f
	}
	return true;
}
