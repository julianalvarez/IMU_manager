
#ifndef _IMU_DRIVER_H
#define _IMU_DRIVER_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "include\imu_hal.h"

/* Defines ********************************************************************/
#define IMU_DIAGNOSTIC_NO_ERROR         (0x00000000)
#define IMU_DIAGNOSTIC_ACCX_ERROR       (0x00000001)
#define IMU_DIAGNOSTIC_ACCY_ERROR       (0x00000002)
#define IMU_DIAGNOSTIC_ACCZ_ERROR       (0x00000004)
#define IMU_DIAGNOSTIC_GYROP_ERROR      (0x00000008)
#define IMU_DIAGNOSTIC_GYROR_ERROR      (0x00000010)
#define IMU_DIAGNOSTIC_GYROY_ERROR      (0x00000020)
#define IMU_DIAGNOSTIC_TEMPP_ERROR      (0x00000040)
#define IMU_DIAGNOSTIC_TEMPR_ERROR      (0x00000080)
#define IMU_DIAGNOSTIC_TEMPY_ERROR      (0x00000100)
#define IMU_DIAGNOSTIC_G_ERROR          (0x00000200)
#define IMU_DIAGNOSTIC_VREF_ERROR       (0x00000400)
#define IMU_DIAGNOSTIC_CALIB_ERROR      (0x00000800)
#define IMU_DIAGNOSTIC_COMM_ERROR       (0x00001000)
#define IMU_DIAGNOSTIC_NO_INIT          (0x00002000)

#define ECUATORIAL_EFFECTIVE_GRAVITY    (9.7803267715f)
#define DEG_TO_RAD                      (3.1415926535897932384626433832795f/180.0f)
#define RAD_TO_DEG                      (180.0f/3.1415926535897932384626433832795f)
/* Enums **********************************************************************/
typedef enum {
  IMU_STATUS_OK                        =  0,
  IMU_STATUS_ERROR_EEPROM              =  1,
  IMU_STATUS_ERROR_COMM                =  2,
  IMU_STATUS_ERROR_BAD_DEVICE_ID       =  3,
  IMU_STATUS_ERROR_BAD_CONFIGURATION   =  4,
  IMU_STATUS_ERROR_BAD_INIT            =  5,
  IMU_STATUS_ERROR_NO_INIT             =  6,
  IMU_STATUS_ERROR_BAD_PARAM_ID        =  7
} imu_op_status_e;

/* Typedefs *******************************************************************/
typedef struct tag_raw_data_struct {
	real32_T				Ax_mpss;
	real32_T				Ay_mpss;
	real32_T				Az_mpss;
	real32_T				Gp_rps;
	real32_T				Gr_rps;
	real32_T				Gy_rps;
} imu_raw_data_struct_t;

typedef struct tag_imu_data_struct {
	real32_T				latAcc_mpss;
	real32_T				longAcc_mpss;
	real32_T				vertAcc_mpss;
	real32_T				pitchRate_rps;
	real32_T				rollRate_rps;
	real32_T				yawRate_rps;
} imu_data_struct_t;

typedef struct tag_imu_temperature_struct {
	real32_T				Ax_Temp_degC;
	real32_T				Ay_Temp_degC;
	real32_T				Az_Temp_degC;
	real32_T				Gp_Temp_degC;
	real32_T				Gr_Temp_degC;
	real32_T				Gy_Temp_degC;
} imu_temperature_struct_t;

typedef struct tag_imu_filter_data_struct {
	real32_T				pitchAngle_rad;
	real32_T				rollAngle_rad;
	real32_T				yawAngle_rad;
} imu_filter_data_struct_t;

typedef imu_op_status_e (*imu_driver_init_ptr) (imu_hal_struct_t *iHal);
typedef imu_op_status_e (*imu_driver_processor_ptr) (void);

typedef imu_op_status_e (*imu_driver_get_diagnostic_ptr) (uint32_t* oDiag);
typedef imu_op_status_e (*imu_driver_get_imu_raw_values_ptr) (imu_raw_data_struct_t* oRawValues);
typedef imu_op_status_e (*imu_driver_get_imu_temperature_ptr) (real32_T* oTemperature);
typedef imu_op_status_e (*imu_driver_get_imu_temperatures_ptr) (imu_temperature_struct_t* oTemperatures);

/**
  * @brief  IMU Driver structure
  *
*/
typedef struct tag_imu_driver_struct {
  imu_driver_init_ptr                       pInit;
  imu_driver_processor_ptr                  pProcessor;
  imu_driver_get_diagnostic_ptr             pDiagnostic;
  imu_driver_get_imu_raw_values_ptr         pGetRawValues;
  imu_driver_get_imu_temperature_ptr        pGetTemperature;
  imu_driver_get_imu_temperatures_ptr       pGetTemperatures;
} imu_driver_struct_t;

/* Externs ********************************************************************/

/* Prototypes *****************************************************************/

#endif /*_IMU_DRIVER_H */
