
#ifndef _IMU_FILTER_H
#define _IMU_FILTER_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "include\imu_driver.h"

/* Defines ********************************************************************/

/* Enums **********************************************************************/

/* Typedefs *******************************************************************/

/* Externs ********************************************************************/

/* Prototypes *****************************************************************/
imu_op_status_e IMU_FILTER_Init(imu_filter_hal_struct_t *iHal);
imu_op_status_e IMU_FILTER_Processor(void);
imu_op_status_e IMU_FILTER_Reset(void);

imu_op_status_e IMU_FILTER_GetValues(imu_filter_data_struct_t* oValues);
imu_op_status_e IMU_FILTER_GetPitchAngle(real32_T* oValue);
imu_op_status_e IMU_FILTER_GetRollAngle(real32_T* oValue);
imu_op_status_e IMU_FILTER_GetYawAngle(real32_T* oValue);

imu_op_status_e IMU_FILTER_SetFilterActive(bool value);
imu_op_status_e IMU_FILTER_GetFilterActive(bool *value);

#endif /*_IMU_FILTER_H */
