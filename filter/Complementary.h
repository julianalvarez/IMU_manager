/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

	@section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_H

#include "types.h"

//---------------------------------------------------------------------------------------------------
// Function declarations
void FILTER_reset(void);

bool FILTER_setGainAcc(float gain);
float FILTER_getGainAcc(void);

bool FILTER_setBiasAlpha(float bias_alpha);
float FILTER_getBiasAlpha(void);

// When the filter is in the steady state, bias estimation will occur (if the
// parameter is enabled).
bool FILTER_getSteadyState(void);

void FILTER_setDoBiasEstimation(bool do_bias_estimation);
bool FILTER_getDoBiasEstimation(void);

void FILTER_setDoAdaptiveGain(bool do_adaptive_gain);
bool FILTER_getDoAdaptiveGain(void);

float FILTER_getAngularVelocityBiasX(void);
float FILTER_getAngularVelocityBiasY(void);
float FILTER_getAngularVelocityBiasZ(void);

// Set the orientation, as a Hamilton Quaternion, of the body frame wrt the
// fixed frame.
void FILTER_setOrientation(float q0, float q1, float q2, float q3);

// Get the orientation, as a Hamilton Quaternion, of the body frame wrt the
// fixed frame.
void FILTER_getOrientation(float* q0, float* q1, float* q2, float* q3);

// Update from accelerometer and gyroscope data.
// [ax, ay, az]: Normalized gravity vector.
// [wx, wy, wz]: Angular veloctiy, in rad / s.
// dt: time delta, in seconds.
void FILTER_update(float ax, float ay, float az, 
            float wx, float wy, float wz,
            float dt);

#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_H
