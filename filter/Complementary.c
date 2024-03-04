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

//---------------------------------------------------------------------------------------------------
// Header files

#include "Complementary.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions


//---------------------------------------------------------------------------------------------------
// Variable definitions
static const float kGravity = 9.7803267715f;
// Bias estimation steady state thresholds
static const float kAngularVelocityThreshold = 0.2f;
static const float kAccelerationThreshold = 0.1f;
static const float kDeltaAngularVelocityThreshold = 0.01f;

// Gain parameter for the complementary filter, belongs in [0, 1].
static float gain_acc_ = 0.01f;

// Bias estimation gain parameter, belongs in [0, 1].
static float bias_alpha_ = 0.01f;

// Parameter whether to do bias estimation or not.
static bool do_bias_estimation_ = true;

// Parameter whether to do adaptive gain or not.
static bool do_adaptive_gain_ = false;

static bool initialized_ = false;
static bool steady_state_ = false;

// The orientation as a Hamilton quaternion (q0 is the scalar). Represents
// the orientation of the fixed frame wrt the body frame.
static float q0_ = 1.0f;
static float q1_ = 0.0f;
static float q2_ = 0.0f;
static float q3_ = 0.0f; 

// Previus angular velocities;
static float wx_prev_ = 0.0f;
static float wy_prev_ = 0.0f;
static float wz_prev_ = 0.0f;

// Bias in angular velocities;
static float wx_bias_ = 0.0f;
static float wy_bias_ = 0.0f;
static float wz_bias_ = 0.0f;

//---------------------------------------------------------------------------------------------------
// Function declarations

// Filter math functions
void updateBiases(float ax, float ay, float az, 
                  float wx, float wy, float wz);

bool checkState(float ax, float ay, float az, 
                float wx, float wy, float wz);

void getPrediction(
    float wx, float wy, float wz, float dt, 
    float* q0_pred, float* q1_pred, float* q2_pred, float* q3_pred);

void getMeasurement(
    float ax, float ay, float az, 
    float* q0_meas, float* q1_meas, float* q2_meas, float* q3_meas);

void getAccCorrection(
    float ax, float ay, float az,
    float p0, float p1, float p2, float p3,
    float* dq0, float* dq1, float* dq2, float* dq3);
    
float getAdaptiveGain(float alpha, float ax, float ay, float az);                   

// Utility math functions:
void normalizeVector(float* x, float* y, float* z);

void normalizeQuaternion(float* q0, float* q1, float* q2, float* q3);

void scaleQuaternion(float gain,
                     float* dq0, float* dq1, float* dq2, float* dq3); 

void invertQuaternion(
    float q0, float q1, float q2, float q3,
    float* q0_inv, float* q1_inv, float* q2_inv, float* q3_inv);

void quaternionMultiplication(float p0, float p1, float p2, float p3,
                              float q0, float q1, float q2, float q3,
                              float* r0, float* r1, float* r2, float* r3);

void rotateVectorByQuaternion(float x, float y, float z,
                              float q0, float q1, float q2, float q3,
                              float* vx, float* vy, float* vz);

//====================================================================================================
// Public Functions
void FILTER_setDoBiasEstimation(bool do_bias_estimation)
{
  do_bias_estimation_ = do_bias_estimation;
}

bool FILTER_getDoBiasEstimation(void)
{
  return do_bias_estimation_;
}

void FILTER_setDoAdaptiveGain(bool do_adaptive_gain)
{
  do_adaptive_gain_ = do_adaptive_gain;
}

bool FILTER_getDoAdaptiveGain(void)
{
  return do_adaptive_gain_;
}

bool FILTER_setGainAcc(float gain)
{
  if (gain >= 0.0f && gain <= 1.0f)
  {
    gain_acc_ = gain;
    return true;
  }
  else
    return false;
}

float FILTER_getGainAcc(void) 
{
  return gain_acc_;
}

bool FILTER_getSteadyState(void) 
{
  return steady_state_;
}

bool FILTER_setBiasAlpha(float bias_alpha)
{
  if (bias_alpha >= 0.0f && bias_alpha <= 1.0f)
  {
    bias_alpha_ = bias_alpha;
    return true;
  }
  else
    return false;
}

float FILTER_getBiasAlpha(void) 
{
  return bias_alpha_;
}

void FILTER_setOrientation(
    float q0, float q1, float q2, float q3) 
{
  // Set the state to inverse (state is fixed wrt body).
  invertQuaternion(q0, q1, q2, q3, &q0_, &q1_, &q2_, &q3_);
}

float FILTER_getAngularVelocityBiasX(void)
{
  return wx_bias_;
}

float FILTER_getAngularVelocityBiasY(void)
{
  return wy_bias_;
}

float FILTER_getAngularVelocityBiasZ(void)
{
  return wz_bias_;
}

void FILTER_reset(void)
{
  initialized_ = false;
  steady_state_ = false;

  q0_ = 1.0f;
  q1_ = 0.0f;
  q2_ = 0.0f;
  q3_ = 0.0f; 

  wx_prev_ = 0.0f;
  wy_prev_ = 0.0f;
  wz_prev_ = 0.0f;

  wx_bias_ = 0.0f;
  wy_bias_ = 0.0f;
  wz_bias_ = 0.0f;
}

void FILTER_update(float ax, float ay, float az, 
                                 float wx, float wy, float wz,
                                 float dt)
{
  float q0_pred, q1_pred, q2_pred, q3_pred;
  float dq0_acc, dq1_acc, dq2_acc, dq3_acc;  
  float gain;
  
  if (!initialized_) 
  {
    // First time - ignore prediction:
    getMeasurement(ax, ay, az,
                   &q0_, &q1_, &q2_, &q3_);
    initialized_ = true;
    return;
  }
  
  // Bias estimation.
  if (do_bias_estimation_)
    updateBiases(ax, ay, az, wx, wy, wz);

  // Prediction.
  getPrediction(wx, wy, wz, dt,
                &q0_pred, &q1_pred, &q2_pred, &q3_pred);   
     
  // Correction (from acc): 
  // q_ = q_pred * [(1-gain) * qI + gain * dq_acc]
  // where qI = identity quaternion
  getAccCorrection(ax, ay, az,
                   q0_pred, q1_pred, q2_pred, q3_pred,
                   &dq0_acc, &dq1_acc, &dq2_acc, &dq3_acc);
  
  if (do_adaptive_gain_)
  {  
    gain = getAdaptiveGain(gain_acc_, ax, ay, az);
    
  }
  else
  {
    gain = gain_acc_;
    
  }

  scaleQuaternion(gain, &dq0_acc, &dq1_acc, &dq2_acc, &dq3_acc);

  quaternionMultiplication(q0_pred, q1_pred, q2_pred, q3_pred,
                           dq0_acc, dq1_acc, dq2_acc, dq3_acc,
                           &q0_, &q1_, &q2_, &q3_);

  normalizeQuaternion(&q0_, &q1_, &q2_, &q3_);
}

bool checkState(float ax, float ay, float az, 
                                     float wx, float wy, float wz)
{
  float acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
  if (fabs(acc_magnitude - kGravity) > kAccelerationThreshold)
    return false;

  if (fabs(wx - wx_prev_) > kDeltaAngularVelocityThreshold ||
      fabs(wy - wy_prev_) > kDeltaAngularVelocityThreshold ||
      fabs(wz - wz_prev_) > kDeltaAngularVelocityThreshold)
    return false;

  if (fabs(wx - wx_bias_) > kAngularVelocityThreshold ||
      fabs(wy - wy_bias_) > kAngularVelocityThreshold ||
      fabs(wz - wz_bias_) > kAngularVelocityThreshold)
    return false;

  return true;
}

void updateBiases(float ax, float ay, float az, 
                                       float wx, float wy, float wz)
{
  steady_state_ = checkState(ax, ay, az, wx, wy, wz);

  if (steady_state_)
  {
    wx_bias_ += bias_alpha_ * (wx - wx_bias_);
    wy_bias_ += bias_alpha_ * (wy - wy_bias_);
    wz_bias_ += bias_alpha_ * (wz - wz_bias_);
  }

  wx_prev_ = wx; 
  wy_prev_ = wy; 
  wz_prev_ = wz;
}

void getPrediction(
    float wx, float wy, float wz, float dt, 
    float* q0_pred, float* q1_pred, float* q2_pred, float* q3_pred)
{
  float wx_unb = wx - wx_bias_;
  float wy_unb = wy - wy_bias_;
  float wz_unb = wz - wz_bias_;

  *q0_pred = q0_ + 0.5f*dt*( wx_unb*q1_ + wy_unb*q2_ + wz_unb*q3_);
  *q1_pred = q1_ + 0.5f*dt*(-wx_unb*q0_ - wy_unb*q3_ + wz_unb*q2_);
  *q2_pred = q2_ + 0.5f*dt*( wx_unb*q3_ - wy_unb*q0_ - wz_unb*q1_);
  *q3_pred = q3_ + 0.5f*dt*(-wx_unb*q2_ + wy_unb*q1_ - wz_unb*q0_);
  
  normalizeQuaternion(q0_pred, q1_pred, q2_pred, q3_pred);
}

void getMeasurement(
    float ax, float ay, float az, 
    float* q0_meas, float* q1_meas, float* q2_meas, float* q3_meas)
{
  // q_acc is the quaternion obtained from the acceleration vector representing 
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
     
  // Normalize acceleration vector.
  normalizeVector(&ax, &ay, &az);

  if (az >=0)
  {
    *q0_meas =  sqrt((az + 1.0f) * 0.5f);	
    *q1_meas = -ay/(2.0f * *q0_meas);
    *q2_meas =  ax/(2.0f * *q0_meas);
    *q3_meas = 0.0f;
  }
  else 
  {
    float X = sqrt((1.0f - az) * 0.5f);
    *q0_meas = -ay/(2.0f * X);
    *q1_meas = X;
    *q2_meas = 0.0f;
    *q3_meas = ax/(2.0f * X);
  }  
}

void getAccCorrection(
  float ax, float ay, float az,
  float p0, float p1, float p2, float p3,
  float* dq0, float* dq1, float* dq2, float* dq3)
{
  float gx, gy, gz;
  
  // Normalize acceleration vector.
  normalizeVector(&ax, &ay, &az);
  
  // Acceleration reading rotated into the world frame by the inverse predicted
  // quaternion (predicted gravity):
  rotateVectorByQuaternion(ax, ay, az,
                           p0, -p1, -p2, -p3, 
                           &gx, &gy, &gz);
  
  // Delta quaternion that rotates the predicted gravity into the real gravity:
  *dq0 =  sqrt((gz + 1.0f) * 0.5f);	
  *dq1 = -gy/(2.0f * *dq0);
  *dq2 =  gx/(2.0f * *dq0);
  *dq3 =  0.0f;
}
 
void FILTER_getOrientation(
    float* q0, float* q1, float* q2, float* q3)
{
  // Return the inverse of the state (state is fixed wrt body).
  invertQuaternion(q0_, q1_, q2_, q3_, q0, q1, q2, q3);
}

float getAdaptiveGain(float alpha, float ax, float ay, float az)
{
  float a_mag = sqrt(ax*ax + ay*ay + az*az);
  float error = fabs(a_mag - kGravity)/kGravity;
  float factor;
  float error1 = 0.1f;
  float error2 = 0.2f;
  float m = 1.0f/(error1 - error2);
  float b = 1.0f - m*error1;
  if (error < error1)
    factor = 1.0f;
  else if (error < error2)
    factor = m*error + b;
  else 
    factor = 0.0f;

  return factor*alpha;
}

// Private functions

void normalizeVector(float* x, float* y, float* z)
{
  float norm = sqrt(*x * *x + *y * *y + *z * *z);

  *x /= norm;
  *y /= norm;
  *z /= norm;
}

void normalizeQuaternion(float* q0, float* q1, float* q2, float* q3)
{
  float norm = sqrt(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
  *q0 /= norm;  
  *q1 /= norm;
  *q2 /= norm;
  *q3 /= norm;
}

void invertQuaternion(
  float q0, float q1, float q2, float q3,
  float* q0_inv, float* q1_inv, float* q2_inv, float* q3_inv)
{
  // Assumes quaternion is normalized.
  *q0_inv = q0;
  *q1_inv = -q1;
  *q2_inv = -q2;
  *q3_inv = -q3;
}

void scaleQuaternion(
  float gain,
  float* dq0, float* dq1, float* dq2, float* dq3)
{
	if (*dq0 < 0.0f)//0.9
  {
    // Slerp (Spherical linear interpolation):
    float angle = acosf(*dq0);
    float A = sinf(angle*(1.0f - gain))/sin(angle);
    float B = sinf(angle * gain)/sin(angle);
    *dq0 = A + B * *dq0;
    *dq1 = B * *dq1;
    *dq2 = B * *dq2;
    *dq3 = B * *dq3;
  }
  else
  {
    // Lerp (Linear interpolation):
    *dq0 = (1.0f - gain) + gain * *dq0;
    *dq1 = gain * *dq1;
    *dq2 = gain * *dq2;
    *dq3 = gain * *dq3;
  }

  normalizeQuaternion(dq0, dq1, dq2, dq3);  
}

void quaternionMultiplication(
  float p0, float p1, float p2, float p3,
  float q0, float q1, float q2, float q3,
  float* r0, float* r1, float* r2, float* r3)
{
  // r = p q
  *r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;
  *r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
  *r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1;
  *r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0;
}

void rotateVectorByQuaternion( 
  float x, float y, float z,
  float q0, float q1, float q2, float q3,
  float* vx, float* vy, float* vz)
{ 
  *vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2.0f*(q1*q2 - q0*q3)*y + 2.0f*(q1*q3 + q0*q2)*z;
  *vy = 2.0f*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2.0f*(q2*q3 - q0*q1)*z;
  *vz = 2.0f*(q1*q3 - q0*q2)*x + 2.0f*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z;
}
