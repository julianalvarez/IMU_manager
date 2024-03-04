
#ifndef _IMU_MANAGER_H
#define _IMU_MANAGER_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"
#include "include\imu_hal.h"
#include "include\imu_driver.h"

/* Defines ********************************************************************/

/* Enums **********************************************************************/
typedef enum {
  IMU_HW_ID_LSM6DSM                      =  0,
  IMU_HW_ID_ADC8344_ADXRS614_ADCL203     =  1,
  IMU_HW_ID_ASM330LHH                    =  2
} imu_hw_id_e;

/* List of possible IMU mounting positions */
/* First word determines the direction of the module's X-Y plane (right-front) */
/* Second word determines the orientation of the module's Z axis (up) */
/* Third word determines the orientation of the module's Y axis (front) */
typedef enum {
  IMU_MOUNTING_POSITION_UNKNOWN                         = 0,  // No rotation is performed
  IMU_MOUNTING_POSITION_VERTICAL_FRONTWARD_UP           = 1,
  IMU_MOUNTING_POSITION_VERTICAL_FRONTWARD_LEFT         = 2,
  IMU_MOUNTING_POSITION_VERTICAL_FRONTWARD_DOWN         = 3,
  IMU_MOUNTING_POSITION_VERTICAL_FRONTWARD_RIGHT        = 4,
  IMU_MOUNTING_POSITION_VERTICAL_LEFTWARD_UP            = 5,
  IMU_MOUNTING_POSITION_VERTICAL_LEFTWARD_FRONT         = 6,
  IMU_MOUNTING_POSITION_VERTICAL_LEFTWARD_DOWN          = 7,
  IMU_MOUNTING_POSITION_VERTICAL_LEFTWARD_BACK          = 8,
  IMU_MOUNTING_POSITION_VERTICAL_BACKWARD_UP            = 9,
  IMU_MOUNTING_POSITION_VERTICAL_BACKWARD_LEFT          = 10,
  IMU_MOUNTING_POSITION_VERTICAL_BACKWARD_DOWN          = 11,
  IMU_MOUNTING_POSITION_VERTICAL_BACKWARD_RIGHT         = 12,
  IMU_MOUNTING_POSITION_VERTICAL_RIGHTWARD_UP           = 13,
  IMU_MOUNTING_POSITION_VERTICAL_RIGHTWARD_FRONT        = 14,
  IMU_MOUNTING_POSITION_VERTICAL_RIGHTWARD_DOWN         = 15,
  IMU_MOUNTING_POSITION_VERTICAL_RIGHTWARD_BACK         = 16,
  IMU_MOUNTING_POSITION_HORIZONTAL_UPWARD_FRONT         = 17,
  IMU_MOUNTING_POSITION_HORIZONTAL_UPWARD_LEFT          = 18,
  IMU_MOUNTING_POSITION_HORIZONTAL_UPWARD_BACK          = 19,
  IMU_MOUNTING_POSITION_HORIZONTAL_UPWARD_RIGHT         = 20,
  IMU_MOUNTING_POSITION_HORIZONTAL_DOWNWARD_FRONT       = 21,
  IMU_MOUNTING_POSITION_HORIZONTAL_DOWNWARD_LEFT        = 22,
  IMU_MOUNTING_POSITION_HORIZONTAL_DOWNWARD_BACK        = 23,
  IMU_MOUNTING_POSITION_HORIZONTAL_DOWNWARD_RIGHT       = 24,
  IMU_MOUNTING_POSITION_CUSTOM                          = 100,  // Rotation is performed according to Mounting Quaternion
  IMU_MOUNTING_POSITION_LAST
} imu_mounting_positions_e;

typedef enum /*!< List of 36 IMU Parameters accessible with Get_Parameter/Set_Parameter */
{
  /* Accelerometer Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GXX				= 0x0210U,    /*!< X Accelerometer Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GXY				= 0x0211U,    /*!< X-Y Accelerometer Cross-Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GXZ				= 0x0212U,    /*!< X-Z Accelerometer Cross-Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GYX				= 0x0213U,    /*!< Y-X Accelerometer Cross-Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GYY				= 0x0214U,    /*!< Y Accelerometer Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GYZ				= 0x0215U,    /*!< Y-Z Accelerometer Cross-Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GZX				= 0x0216U,    /*!< Z-X Accelerometer Cross-Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GZY				= 0x0217U,    /*!< Z-Y Accelerometer Cross-Gain */
  IMU_CALIBRATION_PARAMETER_ACC_GZZ				= 0x0218U,    /*!< Z Accelerometer Gain */
  /* Accelerometer Offset */  
  IMU_CALIBRATION_PARAMETER_ACC_OXT1			= 0x0220U,    /*!< X Accelerometer Offset - Quadratic temperature coefficient */
  IMU_CALIBRATION_PARAMETER_ACC_OXT2			= 0x0221U,    /*!< X Accelerometer Offset - Linear temperature coefficient */
  IMU_CALIBRATION_PARAMETER_ACC_OXT3			= 0x0222U,    /*!< X Accelerometer Offset - Constant coefficient */
  IMU_CALIBRATION_PARAMETER_ACC_OYT1			= 0x0223U,    /*!< Y Accelerometer Offset - Quadratic temperature coefficient */
  IMU_CALIBRATION_PARAMETER_ACC_OYT2			= 0x0224U,    /*!< Y Accelerometer Offset - Linear temperature coefficient */
  IMU_CALIBRATION_PARAMETER_ACC_OYT3			= 0x0225U,    /*!< Y Accelerometer Offset - Constant coefficient */
  IMU_CALIBRATION_PARAMETER_ACC_OZT1			= 0x0226U,    /*!< Z Accelerometer Offset - Quadratic temperature coefficient */
  IMU_CALIBRATION_PARAMETER_ACC_OZT2			= 0x0227U,    /*!< Z Accelerometer Offset - Linear temperature coefficient */
  IMU_CALIBRATION_PARAMETER_ACC_OZT3			= 0x0228U,    /*!< Z Accelerometer Offset - Constant coefficient */
  /* Gyroscope Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GPP				= 0x0310U,    /*!< PitchRate Gyroscope Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GPR				= 0x0311U,    /*!< PitchRate-RollRate Gyroscope Cross-Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GPY				= 0x0312U,    /*!< PitchRate-YawRate Gyroscope Cross-Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GRP				= 0x0313U,    /*!< RollRate-PitchRate Gyroscope Cross-Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GRR				= 0x0314U,    /*!< RollRate Gyroscope Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GRY				= 0x0315U,    /*!< RollRate-YawRate Gyroscope Cross-Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GYP				= 0x0316U,    /*!< YawRate-PitchRate Gyroscope Cross-Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GYR				= 0x0317U,    /*!< YawRate-RollRate Gyroscope Cross-Gain */
  IMU_CALIBRATION_PARAMETER_GYR_GYY				= 0x0318U,    /*!< YawRate Gyroscope Gain */
  /* Gyroscope Offset*/  
  IMU_CALIBRATION_PARAMETER_GYR_OPT1			= 0x0320U,    /*!< PitchRate Gyroscope Offset - Quadratic temperature coefficient */
  IMU_CALIBRATION_PARAMETER_GYR_OPT2			= 0x0321U,    /*!< PitchRate Gyroscope Offset - Linear temperature coefficient */
  IMU_CALIBRATION_PARAMETER_GYR_OPT3			= 0x0322U,    /*!< PitchRate Gyroscope Offset - Constant coefficient */
  IMU_CALIBRATION_PARAMETER_GYR_ORT1			= 0x0323U,    /*!< RollRate Gyroscope Offset - Quadratic temperature coefficient */
  IMU_CALIBRATION_PARAMETER_GYR_ORT2			= 0x0324U,    /*!< RollRate Gyroscope Offset - Linear temperature coefficient */
  IMU_CALIBRATION_PARAMETER_GYR_ORT3			= 0x0325U,    /*!< RollRate Gyroscope Offset - Constant coefficient */
  IMU_CALIBRATION_PARAMETER_GYR_OYT1			= 0x0326U,    /*!< YawRate Gyroscope Offset - Quadratic temperature coefficient */
  IMU_CALIBRATION_PARAMETER_GYR_OYT2			= 0x0327U,    /*!< YawRate Gyroscope Offset - Linear temperature coefficient */
  IMU_CALIBRATION_PARAMETER_GYR_OYT3			= 0x0328U,    /*!< YawRate Gyroscope Offset - Constant coefficient */
  /* */
} imu_calibration_parameters_e;

/* Typedefs *******************************************************************/
typedef struct tag_imu_quaternion_struct {
	real32_T				qW;
	real32_T				qX;
	real32_T				qY;
	real32_T				qZ;
} imu_quaternion_struct_t;

/* Externs ********************************************************************/

/* Prototypes *****************************************************************/
imu_op_status_e IMU_MANAGER_Init(imu_hal_struct_t *iHal, imu_hw_id_e iHWID);
imu_op_status_e IMU_MANAGER_Processor(void);
bool            IMU_MANAGER_IsInitialized(void);
bool            IMU_MANAGER_IsReset(void);

uint32_t        IMU_MANAGER_GetIMUEEPROMSize(void);
bool            IMU_MANAGER_IsAccCalibrationValid(void);
bool            IMU_MANAGER_IsGyrCalibrationValid(void);

imu_op_status_e IMU_MANAGER_GetHWID(imu_hw_id_e* oHWID);
//imu_op_status_e IMU_MANAGER_GetHWSerialNumber(uint32_t* oHWSerialNumber);

//imu_op_status_e IMU_MANAGER_GetTimestamp(uint32_t* oTimestamp);
imu_op_status_e IMU_MANAGER_GetDiagnostic(uint32_t* oDiag);

imu_op_status_e IMU_MANAGER_GetIMURawValues(imu_raw_data_struct_t* oRawValues);
imu_op_status_e IMU_MANAGER_GetIMURawAx(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMURawAy(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMURawAz(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMURawGp(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMURawGr(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMURawGy(real32_T* oValue);

imu_op_status_e IMU_MANAGER_GetIMUValues(imu_data_struct_t* oValues);
imu_op_status_e IMU_MANAGER_GetIMUALat(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUALong(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUAVert(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUGPitchRate(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUGRollRate(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUGYawRate(real32_T* oValue);

imu_op_status_e IMU_MANAGER_GetIMUTemperature(real32_T* oTemperature);
imu_op_status_e IMU_MANAGER_GetIMUTemperatures(imu_temperature_struct_t* oTemperatures);
imu_op_status_e IMU_MANAGER_GetIMUTempAx(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUTempAy(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUTempAz(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUTempGp(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUTempGr(real32_T* oValue);
imu_op_status_e IMU_MANAGER_GetIMUTempGy(real32_T* oValue);

imu_op_status_e IMU_MANAGER_SetCalibrationParameter(imu_calibration_parameters_e id, real32_T value);
imu_op_status_e IMU_MANAGER_GetCalibrationParameter(imu_calibration_parameters_e id, real32_T *value);

imu_op_status_e IMU_MANAGER_SetMountingPosition(uint8_t value);
imu_op_status_e IMU_MANAGER_GetMountingPosition(uint8_t *value);

imu_op_status_e IMU_MANAGER_SetMountingQuaternion(imu_quaternion_struct_t value);
imu_op_status_e IMU_MANAGER_SetMountingQuaternionQw(real32_T value);
imu_op_status_e IMU_MANAGER_SetMountingQuaternionQx(real32_T value);
imu_op_status_e IMU_MANAGER_SetMountingQuaternionQy(real32_T value);
imu_op_status_e IMU_MANAGER_SetMountingQuaternionQz(real32_T value);
imu_op_status_e IMU_MANAGER_SetMountingQuaternionEnd(void);
imu_op_status_e IMU_MANAGER_GetMountingQuaternion(imu_quaternion_struct_t *value);
imu_op_status_e IMU_MANAGER_GetMountingQuaternionQw(real32_T *value);
imu_op_status_e IMU_MANAGER_GetMountingQuaternionQx(real32_T *value);
imu_op_status_e IMU_MANAGER_GetMountingQuaternionQy(real32_T *value);
imu_op_status_e IMU_MANAGER_GetMountingQuaternionQz(real32_T *value);

imu_op_status_e IMU_MANAGER_RestoreDefaultConfigurationAcc(void);
imu_op_status_e IMU_MANAGER_RestoreDefaultConfigurationGyr(void);
imu_op_status_e IMU_MANAGER_RestoreDefaultConfigurationFull(void);
imu_op_status_e IMU_MANAGER_PersistConfiguration(void);

#endif /*_IMU_MANAGER_H */
