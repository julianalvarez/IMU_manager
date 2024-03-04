
/* Includes ------------------------------------------------------------------*/
#include "include\imu_filter.h"
#include "include\imu_manager.h"

#include "filter\Complementary.h"
#include "math.h"

/* Defines ********************************************************************/

/* Typedefs *******************************************************************/

/* Globals ********************************************************************/

/* Statics ********************************************************************/
static bool initialized = false;
static bool filter_active = false;

static uint32_t update_rate_ms = 10U;
static uint32_t last_update = 0U;

static imu_filter_hal_struct_t imu_filter_hal;
static imu_filter_data_struct_t imu_filter_data;

/* Private functions ---------------------------------------------------------*/
void _QuatToEuler(float q0, float q1, float q2, float q3, float *pitch, float *roll, float *yaw);

/* Public functions ---------------------------------------------------------*/
imu_op_status_e IMU_FILTER_Init(imu_filter_hal_struct_t *iHal)
{
  if (initialized)
      return IMU_STATUS_OK;
  
  imu_filter_hal.pTimeGet = iHal->pTimeGet;
  
  FILTER_reset();
  
  initialized = true;
  filter_active = true;
    
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_FILTER_Processor(void)
{
  uint32_t current_time;
  float q0, q1, q2, q3;
  imu_data_struct_t data;
  imu_op_status_e status;
  
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  if (!filter_active)
    return IMU_STATUS_OK;
    
  current_time = imu_filter_hal.pTimeGet();
  
  if (current_time < (last_update + update_rate_ms))
    return IMU_STATUS_OK;
  
  status = IMU_MANAGER_GetIMUValues(&data);
  
  if (status != IMU_STATUS_OK)
    return status;

  // Adapt to FILTER frame of reference
  // x - Forward  - Roll
  // y - Left     - Pitch
  // z - Up       - Yaw
  FILTER_update(-data.longAcc_mpss, data.latAcc_mpss, -data.vertAcc_mpss, 
              data.rollRate_rps, -data.pitchRate_rps, data.yawRate_rps,
              update_rate_ms * 0.001f);

  FILTER_getOrientation(&q0, &q1, &q2, &q3);
  _QuatToEuler(q0, q1, q2, q3, &(imu_filter_data.rollAngle_rad), &(imu_filter_data.pitchAngle_rad), &(imu_filter_data.yawAngle_rad));
  
  imu_filter_data.pitchAngle_rad *= -1.0f;
  
  last_update += update_rate_ms;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_FILTER_Reset(void)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
    
  FILTER_reset();
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_FILTER_GetValues(imu_filter_data_struct_t* oValues)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  oValues->pitchAngle_rad = imu_filter_data.pitchAngle_rad;
  oValues->rollAngle_rad = imu_filter_data.rollAngle_rad;
  oValues->yawAngle_rad = imu_filter_data.yawAngle_rad;
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_FILTER_GetPitchAngle(real32_T* oValue)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  *oValue = imu_filter_data.pitchAngle_rad;
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_FILTER_GetRollAngle(real32_T* oValue)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  *oValue = imu_filter_data.rollAngle_rad;
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_FILTER_GetYawAngle(real32_T* oValue)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  *oValue = imu_filter_data.yawAngle_rad;
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_FILTER_SetFilterActive(bool value)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  filter_active = value;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_FILTER_GetFilterActive(bool *value)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  *value = filter_active;
  
  return IMU_STATUS_OK;
}

void _QuatToEuler(float q0, float q1, float q2, float q3, float *pitch, float *roll, float *yaw)
{
  float sqw;
	float sqx;
	float sqy;
	float sqz;
	
	sqw = q0 * q0;
	sqx = q1 * q1;
	sqy = q2 * q2;
	sqz = q3 * q3;
	
	*pitch = atan2f(2.0f * ( q2 * q3 + q1 * q0 ) , ( -sqx - sqy + sqz + sqw ));
	*roll = asinf(-2.0f * ( q1 * q3 - q2 * q0 ));
	*yaw = atan2f(2.0f * ( q1 * q2 + q3 * q0 ) , (  sqx - sqy - sqz + sqw ));
	
	return;
}
