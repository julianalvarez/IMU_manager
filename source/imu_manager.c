
/* Includes ------------------------------------------------------------------*/
#include "include\imu_manager.h"
#include "include\imu_filter.h"
#include "include\imu_canmsg.h"
#include "math.h"

#if defined(SYSTEM_STEER)
#include "drivers\lsm6dsm\lsm6dsm.h"
#include "drivers\imu_adc8344\imu_adc8344.h"
#include "drivers\asm330lhh\asm330lhh.h"
#elif defined(SYSTEM_ECUROW) || defined(SYSTEM_SMARTANTENNA)
#include "drivers\lsm6dsm\lsm6dsm.h"
#elif defined(SYSTEM_SBOX7)
#include "drivers\lsm6dsm\lsm6dsm.h"
#include "drivers\asm330lhh\asm330lhh.h"
#else
#warning "No module defined"
#endif

/* Defines ********************************************************************/
#define IMU_PARAMETER_VERSION_1                     (1U)
#define IMU_PARAMETER_VERSION_2                     (2U)

#define IMU_PARAMETER_VERSION_1_VERSION_SIZE        (1U)
#define IMU_PARAMETER_VERSION_1_DEFAULT_FLAG_SIZE   (1U)
#define IMU_PARAMETER_VERSION_1_RESERVED_1_SIZE     (2U)
#define IMU_PARAMETER_VERSION_1_CALIBRATION_SIZE    (144U) // (36U*sizeof(real32_T))
#define IMU_PARAMETER_VERSION_1_RESERVED_2_SIZE     (19U)
#define IMU_PARAMETER_VERSION_1_CHECKSUM_SIZE       (1U)
#define IMU_PARAMETER_VERSION_1_USED_SIZE           (IMU_PARAMETER_VERSION_1_VERSION_SIZE + IMU_PARAMETER_VERSION_1_DEFAULT_FLAG_SIZE + IMU_PARAMETER_VERSION_1_CALIBRATION_SIZE + IMU_PARAMETER_VERSION_1_CHECKSUM_SIZE) // 147U  
//#define IMU_PARAMETER_VERSION_1_TOTAL_SIZE          (IMU_PARAMETER_VERSION_1_USED_SIZE + IMU_PARAMETER_VERSION_1_RESERVED_1_SIZE + IMU_PARAMETER_VERSION_1_RESERVED_2_SIZE) // 168U
#define IMU_PARAMETER_VERSION_1_TOTAL_SIZE          (sizeof(imu_calib_data_struct_t)) // 168U

#define ACC_DEFAULT_FLAG (0x1U<<0)
#define GYR_DEFAULT_FLAG (0x1U<<1)

/* Typedefs *******************************************************************/
typedef struct {
  uint8_t imu_parameter_version;
  uint8_t imu_parameter_default_flag;
  /* Reserved 1*/
  uint8_t Rvd1[IMU_PARAMETER_VERSION_1_RESERVED_1_SIZE];
  /* accelerometer gain */
  real32_T imu_parameter_acc_gxx;
  real32_T imu_parameter_acc_gxy;
  real32_T imu_parameter_acc_gxz;
  real32_T imu_parameter_acc_gyx;
  real32_T imu_parameter_acc_gyy;
  real32_T imu_parameter_acc_gyz;
  real32_T imu_parameter_acc_gzx;
  real32_T imu_parameter_acc_gzy;
  real32_T imu_parameter_acc_gzz;
  /* accelerometer offset */  
  real32_T imu_parameter_acc_oxt1;
  real32_T imu_parameter_acc_oxt2;
  real32_T imu_parameter_acc_oxt3;
  real32_T imu_parameter_acc_oyt1;
  real32_T imu_parameter_acc_oyt2;
  real32_T imu_parameter_acc_oyt3;
  real32_T imu_parameter_acc_ozt1;
  real32_T imu_parameter_acc_ozt2;
  real32_T imu_parameter_acc_ozt3;
  /* gyroscope gain */
  real32_T imu_parameter_gyr_gpp;
  real32_T imu_parameter_gyr_gpr;
  real32_T imu_parameter_gyr_gpy;
  real32_T imu_parameter_gyr_grp;
  real32_T imu_parameter_gyr_grr;
  real32_T imu_parameter_gyr_gry;
  real32_T imu_parameter_gyr_gyp;
  real32_T imu_parameter_gyr_gyr;
  real32_T imu_parameter_gyr_gyy;
  /* gyroscope offset */
  real32_T imu_parameter_gyr_opt1;
  real32_T imu_parameter_gyr_opt2;
  real32_T imu_parameter_gyr_opt3;
  real32_T imu_parameter_gyr_ort1;
  real32_T imu_parameter_gyr_ort2;
  real32_T imu_parameter_gyr_ort3;
  real32_T imu_parameter_gyr_oyt1;
  real32_T imu_parameter_gyr_oyt2;
  real32_T imu_parameter_gyr_oyt3;
  /* Reserved */
  uint8_t Rvd2[IMU_PARAMETER_VERSION_1_RESERVED_2_SIZE];
  /* Checksum */
  uint8_t checksum;
} imu_calib_data_struct_t;

/* Globals ********************************************************************/

/* Statics ********************************************************************/
static bool initialized = false;
static bool acc_calibration_good = false;
static bool gyr_calibration_good = false;
static imu_hw_id_e hw_id;

static imu_hal_struct_t imu_hal;
static imu_driver_struct_t imu_driver;

static imu_calib_data_struct_t imu_calib_data;
static imu_mounting_positions_e imu_mounting_position = IMU_MOUNTING_POSITION_UNKNOWN;
static imu_quaternion_struct_t imu_quaternion = {1.0, 0.0, 0.0, 0.0};
static real32_T imu_rot_matrix[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
static bool imu_custom_rot_valid = false;
static bool imu_filer_isReset;

/* Private functions ---------------------------------------------------------*/
bool _checkEEPROM(uint8_t* pData);
void _setDefaultCalib(imu_calib_data_struct_t* pData);
void _setDefaultGyrCalib(imu_calib_data_struct_t* pData);
void _setDefaultAccCalib(imu_calib_data_struct_t* pData);
//bool _checkCalib(imu_calib_data_struct_t* pData);
void _calcChecksum(uint8_t* iData, uint8_t* oChecksum, bool* oNonZeroCheck);
real32_T* _getParamPtr(imu_calibration_parameters_e id);
bool _isAccParameter(imu_calibration_parameters_e id);
bool _isGyrParameter(imu_calibration_parameters_e id);
void _applyCalibration(imu_raw_data_struct_t* pData, imu_temperature_struct_t* pTemperature);

void _applyRotation(imu_raw_data_struct_t* pRawData, imu_data_struct_t* pData);
void _applyCustomRotation(imu_raw_data_struct_t* pRawData, imu_data_struct_t* pData);
bool _quat2rotM(imu_quaternion_struct_t* quat, real32_T rotM[][3]);
bool _proccesNewQuat(void);
void _Reset_Filter(void);

bool _checkEEPROM(uint8_t* pData)
{
  uint8_t checksum;
  bool nonZeroCheck;
  
  _calcChecksum(pData, &checksum, &nonZeroCheck);
  
  return (nonZeroCheck && (checksum == pData[IMU_PARAMETER_VERSION_1_TOTAL_SIZE - 1]));
}

void _setDefaultCalib(imu_calib_data_struct_t* pData)
{
  int i;
  bool tmp;
  
  for (i = 0; i < IMU_PARAMETER_VERSION_1_TOTAL_SIZE; i++) {
    *((uint8_t*)pData + i) = 0x00;
  }
  
  /* Version */
  pData->imu_parameter_version = IMU_PARAMETER_VERSION_2;
  
  /* Default flag */
  pData->imu_parameter_default_flag = 0xFF;
  
  /* Reserved 1 */
  for (i = 0; i < IMU_PARAMETER_VERSION_1_RESERVED_1_SIZE; i++) {
    pData->Rvd1[i] = 0;
  }
  
  _setDefaultAccCalib(pData);
  
  _setDefaultGyrCalib(pData);
  
  /* Reserved 2 */
  for (i = 0; i < IMU_PARAMETER_VERSION_1_RESERVED_2_SIZE; i++) {
    pData->Rvd2[i] = 0;
  }
  
  /* Checksum */
  _calcChecksum((uint8_t*)pData, &pData->checksum, &tmp);
}

void _setDefaultAccCalib(imu_calib_data_struct_t* pData)
{
  pData->imu_parameter_default_flag |= ACC_DEFAULT_FLAG;
  acc_calibration_good = false;
  
  /* accelerometer gain */
  pData->imu_parameter_acc_gxx = 1;
  pData->imu_parameter_acc_gxy = 0;
  pData->imu_parameter_acc_gxz = 0;
  pData->imu_parameter_acc_gyx = 0;
  pData->imu_parameter_acc_gyy = 1;
  pData->imu_parameter_acc_gyz = 0;
  pData->imu_parameter_acc_gzx = 0;
  pData->imu_parameter_acc_gzy = 0;
  pData->imu_parameter_acc_gzz = 1;
  
  /* accelerometer offset */  
  pData->imu_parameter_acc_oxt1 = 0;
  pData->imu_parameter_acc_oxt2 = 0;
  pData->imu_parameter_acc_oxt3 = 0;
  pData->imu_parameter_acc_oyt1 = 0;
  pData->imu_parameter_acc_oyt2 = 0;
  pData->imu_parameter_acc_oyt3 = 0;
  pData->imu_parameter_acc_ozt1 = 0;
  pData->imu_parameter_acc_ozt2 = 0;
  pData->imu_parameter_acc_ozt3 = 0;
}

void _setDefaultGyrCalib(imu_calib_data_struct_t* pData)
{
  pData->imu_parameter_default_flag |= GYR_DEFAULT_FLAG;
  gyr_calibration_good = false;
  
  /* gyroscope gain */
  pData->imu_parameter_gyr_gpp = 1;
  pData->imu_parameter_gyr_gpr = 0;
  pData->imu_parameter_gyr_gpy = 0;
  pData->imu_parameter_gyr_grp = 0;
  pData->imu_parameter_gyr_grr = 1;
  pData->imu_parameter_gyr_gry = 0;
  pData->imu_parameter_gyr_gyp = 0;
  pData->imu_parameter_gyr_gyr = 0;
  pData->imu_parameter_gyr_gyy = 1;
  
  /* gyroscope offset*/  
  pData->imu_parameter_gyr_opt1 = 0;
  pData->imu_parameter_gyr_opt2 = 0;
  pData->imu_parameter_gyr_opt3 = 0;
  pData->imu_parameter_gyr_ort1 = 0;
  pData->imu_parameter_gyr_ort2 = 0;
  pData->imu_parameter_gyr_ort3 = 0;
  pData->imu_parameter_gyr_oyt1 = 0;
  pData->imu_parameter_gyr_oyt2 = 0;
  pData->imu_parameter_gyr_oyt3 = 0;
}

//bool _checkCalib(imu_calib_data_struct_t* pData);

void _calcChecksum(uint8_t* iData, uint8_t* oChecksum, bool* oNonZeroCheck)
{
  int i;
  *oChecksum = 0;
	*oNonZeroCheck = false;
  
	for (i = 0; i < IMU_PARAMETER_VERSION_1_TOTAL_SIZE - 1; i++)
	{
		*oChecksum ^= iData[i];
		*oNonZeroCheck |= (iData[i] != 0);
	}
}

real32_T* _getParamPtr(imu_calibration_parameters_e id)
{
  real32_T* pParam = NULL;
  
  switch (id) {
    case IMU_CALIBRATION_PARAMETER_ACC_GXX:
      pParam = &imu_calib_data.imu_parameter_acc_gxx;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_GXY:
      pParam = &imu_calib_data.imu_parameter_acc_gxy;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_GXZ:
      pParam = &imu_calib_data.imu_parameter_acc_gxz;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_GYX:
      pParam = &imu_calib_data.imu_parameter_acc_gyx;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_GYY:
      pParam = &imu_calib_data.imu_parameter_acc_gyy;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_GYZ:
      pParam = &imu_calib_data.imu_parameter_acc_gyz;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_GZX:
      pParam = &imu_calib_data.imu_parameter_acc_gzx;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_GZY:
      pParam = &imu_calib_data.imu_parameter_acc_gzy;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_GZZ:
      pParam = &imu_calib_data.imu_parameter_acc_gzz;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OXT1:
      pParam = &imu_calib_data.imu_parameter_acc_oxt1;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OXT2:
      pParam = &imu_calib_data.imu_parameter_acc_oxt2;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OXT3:
      pParam = &imu_calib_data.imu_parameter_acc_oxt3;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OYT1:
      pParam = &imu_calib_data.imu_parameter_acc_oyt1;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OYT2:
      pParam = &imu_calib_data.imu_parameter_acc_oyt2;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OYT3:
      pParam = &imu_calib_data.imu_parameter_acc_oyt3;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OZT1:
      pParam = &imu_calib_data.imu_parameter_acc_ozt1;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OZT2:
      pParam = &imu_calib_data.imu_parameter_acc_ozt2;
      break;
    case IMU_CALIBRATION_PARAMETER_ACC_OZT3:
      pParam = &imu_calib_data.imu_parameter_acc_ozt3;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GPP:
      pParam = &imu_calib_data.imu_parameter_gyr_gpp;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GPR:
      pParam = &imu_calib_data.imu_parameter_gyr_gpr;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GPY:
      pParam = &imu_calib_data.imu_parameter_gyr_gpy;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GRP:
      pParam = &imu_calib_data.imu_parameter_gyr_grp;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GRR:
      pParam = &imu_calib_data.imu_parameter_gyr_grr;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GRY:
      pParam = &imu_calib_data.imu_parameter_gyr_gry;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GYP:
      pParam = &imu_calib_data.imu_parameter_gyr_gyp;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GYR:
      pParam = &imu_calib_data.imu_parameter_gyr_gyr;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_GYY:
      pParam = &imu_calib_data.imu_parameter_gyr_gyy;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_OPT1:
      pParam = &imu_calib_data.imu_parameter_gyr_opt1;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_OPT2:
      pParam = &imu_calib_data.imu_parameter_gyr_opt2;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_OPT3:
      pParam = &imu_calib_data.imu_parameter_gyr_opt3;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_ORT1:
      pParam = &imu_calib_data.imu_parameter_gyr_ort1;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_ORT2:
      pParam = &imu_calib_data.imu_parameter_gyr_ort2;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_ORT3:
      pParam = &imu_calib_data.imu_parameter_gyr_ort3;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_OYT1:
      pParam = &imu_calib_data.imu_parameter_gyr_oyt1;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_OYT2:
      pParam = &imu_calib_data.imu_parameter_gyr_oyt2;
      break;
    case IMU_CALIBRATION_PARAMETER_GYR_OYT3:
      pParam = &imu_calib_data.imu_parameter_gyr_oyt3;
      break;
  }
  
  return pParam;  
}

bool _isAccParameter(imu_calibration_parameters_e id)
{
  bool ret;
  
  switch (id) {
  case IMU_CALIBRATION_PARAMETER_ACC_GXX:
  case IMU_CALIBRATION_PARAMETER_ACC_GXY:
  case IMU_CALIBRATION_PARAMETER_ACC_GXZ:
  case IMU_CALIBRATION_PARAMETER_ACC_GYX:
  case IMU_CALIBRATION_PARAMETER_ACC_GYY:
  case IMU_CALIBRATION_PARAMETER_ACC_GYZ:
  case IMU_CALIBRATION_PARAMETER_ACC_GZX:
  case IMU_CALIBRATION_PARAMETER_ACC_GZY:
  case IMU_CALIBRATION_PARAMETER_ACC_GZZ:
  case IMU_CALIBRATION_PARAMETER_ACC_OXT1:
  case IMU_CALIBRATION_PARAMETER_ACC_OXT2:
  case IMU_CALIBRATION_PARAMETER_ACC_OXT3:
  case IMU_CALIBRATION_PARAMETER_ACC_OYT1:
  case IMU_CALIBRATION_PARAMETER_ACC_OYT2:
  case IMU_CALIBRATION_PARAMETER_ACC_OYT3:
  case IMU_CALIBRATION_PARAMETER_ACC_OZT1:
  case IMU_CALIBRATION_PARAMETER_ACC_OZT2:
  case IMU_CALIBRATION_PARAMETER_ACC_OZT3:
    ret = true;
    break;
  default:
    ret = false;
    break;  
  }
  
  return ret;  
}

bool _isGyrParameter(imu_calibration_parameters_e id)
{
  bool ret;
  
  switch (id) {
  case IMU_CALIBRATION_PARAMETER_GYR_GPP:
  case IMU_CALIBRATION_PARAMETER_GYR_GPR:
  case IMU_CALIBRATION_PARAMETER_GYR_GPY:
  case IMU_CALIBRATION_PARAMETER_GYR_GRP:
  case IMU_CALIBRATION_PARAMETER_GYR_GRR:
  case IMU_CALIBRATION_PARAMETER_GYR_GRY:
  case IMU_CALIBRATION_PARAMETER_GYR_GYP:
  case IMU_CALIBRATION_PARAMETER_GYR_GYR:
  case IMU_CALIBRATION_PARAMETER_GYR_GYY:
  case IMU_CALIBRATION_PARAMETER_GYR_OPT1:
  case IMU_CALIBRATION_PARAMETER_GYR_OPT2:
  case IMU_CALIBRATION_PARAMETER_GYR_OPT3:
  case IMU_CALIBRATION_PARAMETER_GYR_ORT1:
  case IMU_CALIBRATION_PARAMETER_GYR_ORT2:
  case IMU_CALIBRATION_PARAMETER_GYR_ORT3:
  case IMU_CALIBRATION_PARAMETER_GYR_OYT1:
  case IMU_CALIBRATION_PARAMETER_GYR_OYT2:
  case IMU_CALIBRATION_PARAMETER_GYR_OYT3:
    ret = true;
    break;
  default:
    ret = false;
    break;  
  }
  
  return ret;  
}

void _applyCalibration(imu_raw_data_struct_t* pData, imu_temperature_struct_t* pTemperature)
{
  // Compensate Accelerometer offset and scale
  pData->Ax_mpss *= imu_calib_data.imu_parameter_acc_gxx;
  pData->Ax_mpss += imu_calib_data.imu_parameter_acc_oxt3;

  pData->Ay_mpss *= imu_calib_data.imu_parameter_acc_gyy;
  pData->Ay_mpss += imu_calib_data.imu_parameter_acc_oyt3;

  pData->Az_mpss *= imu_calib_data.imu_parameter_acc_gzz;
  pData->Az_mpss += imu_calib_data.imu_parameter_acc_ozt3;
  
  // Compensate Gyro bias vs. Temperature
  //pData->Gp_rps += imu_calib_data.imu_parameter_gyr_opt1 * pTemperature->Gp_Temp_degC * pTemperature->Gp_Temp_degC + imu_calib_data.imu_parameter_gyr_opt2 * pTemperature->Gp_Temp_degC + imu_calib_data.imu_parameter_gyr_opt3;
  //pData->Gr_rps += imu_calib_data.imu_parameter_gyr_ort1 * pTemperature->Gr_Temp_degC * pTemperature->Gr_Temp_degC + imu_calib_data.imu_parameter_gyr_ort2 * pTemperature->Gr_Temp_degC + imu_calib_data.imu_parameter_gyr_ort3;
  //pData->Gy_rps += imu_calib_data.imu_parameter_gyr_oyt1 * pTemperature->Gy_Temp_degC * pTemperature->Gy_Temp_degC + imu_calib_data.imu_parameter_gyr_oyt2 * pTemperature->Gy_Temp_degC + imu_calib_data.imu_parameter_gyr_oyt3;
}

void _applyRotation(imu_raw_data_struct_t* pRawData, imu_data_struct_t* pData)
{  
  switch (imu_mounting_position) {
    case IMU_MOUNTING_POSITION_VERTICAL_FRONTWARD_UP:
      pData->latAcc_mpss = -pRawData->Ax_mpss;
      pData->longAcc_mpss = pRawData->Az_mpss;
      pData->vertAcc_mpss = pRawData->Ay_mpss;
      pData->pitchRate_rps = -pRawData->Gp_rps;
      pData->rollRate_rps = pRawData->Gy_rps;
      pData->yawRate_rps = pRawData->Gr_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_FRONTWARD_LEFT:  //Add 2 (2)
      pData->latAcc_mpss = -pRawData->Ay_mpss;
      pData->longAcc_mpss = pRawData->Az_mpss;
      pData->vertAcc_mpss = -pRawData->Ax_mpss;
      pData->pitchRate_rps = -pRawData->Gr_rps;
      pData->rollRate_rps = pRawData->Gy_rps;
      pData->yawRate_rps = -pRawData->Gp_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_FRONTWARD_DOWN:
      pData->latAcc_mpss = pRawData->Ax_mpss;
      pData->longAcc_mpss = pRawData->Az_mpss;
      pData->vertAcc_mpss = -pRawData->Ay_mpss;
      pData->pitchRate_rps = pRawData->Gp_rps;
      pData->rollRate_rps = pRawData->Gy_rps;
      pData->yawRate_rps = -pRawData->Gr_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_FRONTWARD_RIGHT:  //Add 3(4)
      pData->latAcc_mpss = pRawData->Ay_mpss;
      pData->longAcc_mpss = pRawData->Az_mpss;
      pData->vertAcc_mpss = pRawData->Ax_mpss;
      pData->pitchRate_rps = pRawData->Gr_rps;
      pData->rollRate_rps = pRawData->Gy_rps;
      pData->yawRate_rps = pRawData->Gp_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_LEFTWARD_UP:
      pData->latAcc_mpss = -pRawData->Az_mpss;
      pData->longAcc_mpss = -pRawData->Ax_mpss;
      pData->vertAcc_mpss = pRawData->Ay_mpss;
      pData->pitchRate_rps = -pRawData->Gy_rps;
      pData->rollRate_rps = -pRawData->Gp_rps;
      pData->yawRate_rps = pRawData->Gr_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_LEFTWARD_FRONT:  //Add 10 (6)
      pData->latAcc_mpss = -pRawData->Az_mpss;
      pData->longAcc_mpss = pRawData->Ay_mpss;
      pData->vertAcc_mpss = pRawData->Ax_mpss;
      pData->pitchRate_rps = -pRawData->Gy_rps;
      pData->rollRate_rps = pRawData->Gr_rps;
      pData->yawRate_rps = pRawData->Gp_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_LEFTWARD_DOWN:  //Add 7 (7)
      pData->latAcc_mpss = -pRawData->Az_mpss;
      pData->longAcc_mpss = pRawData->Ax_mpss;
      pData->vertAcc_mpss = -pRawData->Ay_mpss;
      pData->pitchRate_rps = -pRawData->Gy_rps;
      pData->rollRate_rps = pRawData->Gp_rps;
      pData->yawRate_rps = -pRawData->Gr_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_LEFTWARD_BACK:  //Add 4 (8)
      pData->latAcc_mpss = -pRawData->Az_mpss;
      pData->longAcc_mpss = -pRawData->Ay_mpss;
      pData->vertAcc_mpss = -pRawData->Ax_mpss;
      pData->pitchRate_rps = -pRawData->Gy_rps;
      pData->rollRate_rps = -pRawData->Gr_rps;
      pData->yawRate_rps = -pRawData->Gp_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_BACKWARD_UP:  //Add 1 (9)
      pData->latAcc_mpss = pRawData->Ax_mpss;
      pData->longAcc_mpss = -pRawData->Az_mpss;
      pData->vertAcc_mpss = pRawData->Ay_mpss;
      pData->pitchRate_rps = pRawData->Gp_rps;
      pData->rollRate_rps = -pRawData->Gy_rps;
      pData->yawRate_rps = pRawData->Gr_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_BACKWARD_LEFT:  //Add 5 (10)
      pData->latAcc_mpss = -pRawData->Ay_mpss;
      pData->longAcc_mpss = -pRawData->Az_mpss;
      pData->vertAcc_mpss = pRawData->Ax_mpss;
      pData->pitchRate_rps = -pRawData->Gr_rps;
      pData->rollRate_rps = -pRawData->Gy_rps;
      pData->yawRate_rps = pRawData->Gp_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_BACKWARD_DOWN:  //Add 8 (11)
      pData->latAcc_mpss = -pRawData->Ax_mpss;
      pData->longAcc_mpss = -pRawData->Az_mpss;
      pData->vertAcc_mpss = -pRawData->Ay_mpss;
      pData->pitchRate_rps = -pRawData->Gp_rps;
      pData->rollRate_rps = -pRawData->Gy_rps;
      pData->yawRate_rps = -pRawData->Gr_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_BACKWARD_RIGHT:  //Add 11 (12)
      pData->latAcc_mpss = pRawData->Ay_mpss;
      pData->longAcc_mpss = -pRawData->Az_mpss;
      pData->vertAcc_mpss = -pRawData->Ax_mpss;
      pData->pitchRate_rps = pRawData->Gr_rps;
      pData->rollRate_rps = -pRawData->Gy_rps;
      pData->yawRate_rps = -pRawData->Gp_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_RIGHTWARD_UP:
      pData->latAcc_mpss = pRawData->Az_mpss;
      pData->longAcc_mpss = pRawData->Ax_mpss;
      pData->vertAcc_mpss = pRawData->Ay_mpss;
      pData->pitchRate_rps = pRawData->Gy_rps;
      pData->rollRate_rps = pRawData->Gp_rps;
      pData->yawRate_rps = pRawData->Gr_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_RIGHTWARD_FRONT:  //Add 6 (14)
      pData->latAcc_mpss = pRawData->Az_mpss;
      pData->longAcc_mpss = pRawData->Ay_mpss;
      pData->vertAcc_mpss = -pRawData->Ax_mpss;
      pData->pitchRate_rps = pRawData->Gy_rps;
      pData->rollRate_rps = pRawData->Gr_rps;
      pData->yawRate_rps = -pRawData->Gp_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_RIGHTWARD_DOWN:  //Add 9 (15)
      pData->latAcc_mpss = pRawData->Az_mpss;
      pData->longAcc_mpss = -pRawData->Ax_mpss;
      pData->vertAcc_mpss = -pRawData->Ay_mpss;
      pData->pitchRate_rps = pRawData->Gy_rps;
      pData->rollRate_rps = -pRawData->Gp_rps;
      pData->yawRate_rps = -pRawData->Gr_rps;
    break;
    case IMU_MOUNTING_POSITION_VERTICAL_RIGHTWARD_BACK:  //Add 12 (16)
      pData->latAcc_mpss = pRawData->Az_mpss;
      pData->longAcc_mpss = -pRawData->Ay_mpss;
      pData->vertAcc_mpss = pRawData->Ax_mpss;
      pData->pitchRate_rps = pRawData->Gy_rps;
      pData->rollRate_rps = -pRawData->Gr_rps;
      pData->yawRate_rps = pRawData->Gp_rps;
    break;
    case IMU_MOUNTING_POSITION_HORIZONTAL_UPWARD_BACK:
      pData->latAcc_mpss = -pRawData->Ax_mpss;
      pData->longAcc_mpss = -pRawData->Ay_mpss;
      pData->vertAcc_mpss = pRawData->Az_mpss;
      pData->pitchRate_rps = -pRawData->Gp_rps;
      pData->rollRate_rps = -pRawData->Gr_rps;
      pData->yawRate_rps = pRawData->Gy_rps;
    break;
    case IMU_MOUNTING_POSITION_HORIZONTAL_UPWARD_LEFT:  //Add 13 (18)
      pData->latAcc_mpss = -pRawData->Ay_mpss;
      pData->longAcc_mpss = pRawData->Ax_mpss;
      pData->vertAcc_mpss = pRawData->Az_mpss;
      pData->pitchRate_rps = -pRawData->Gr_rps;
      pData->rollRate_rps = pRawData->Gp_rps;
      pData->yawRate_rps = pRawData->Gy_rps;
    break;
    case IMU_MOUNTING_POSITION_HORIZONTAL_UPWARD_FRONT:
      pData->latAcc_mpss = pRawData->Ax_mpss;
      pData->longAcc_mpss = pRawData->Ay_mpss;
      pData->vertAcc_mpss = pRawData->Az_mpss;
      pData->pitchRate_rps = pRawData->Gp_rps;
      pData->rollRate_rps = pRawData->Gr_rps;
      pData->yawRate_rps = pRawData->Gy_rps;
    break;
    case IMU_MOUNTING_POSITION_HORIZONTAL_UPWARD_RIGHT:  //Add 15 (20)
      pData->latAcc_mpss = pRawData->Ay_mpss;
      pData->longAcc_mpss = -pRawData->Ax_mpss;
      pData->vertAcc_mpss = pRawData->Az_mpss;
      pData->pitchRate_rps = pRawData->Gr_rps;
      pData->rollRate_rps = -pRawData->Gp_rps;
      pData->yawRate_rps = pRawData->Gy_rps;
    break;
    case IMU_MOUNTING_POSITION_HORIZONTAL_DOWNWARD_FRONT:
      pData->latAcc_mpss = -pRawData->Ax_mpss;
      pData->longAcc_mpss = pRawData->Ay_mpss;
      pData->vertAcc_mpss = -pRawData->Az_mpss;
      pData->pitchRate_rps = -pRawData->Gp_rps;
      pData->rollRate_rps = pRawData->Gr_rps;
      pData->yawRate_rps = -pRawData->Gy_rps;
    break;
    case IMU_MOUNTING_POSITION_HORIZONTAL_DOWNWARD_LEFT:
      pData->latAcc_mpss = -pRawData->Ay_mpss;
      pData->longAcc_mpss = -pRawData->Ax_mpss;
      pData->vertAcc_mpss = -pRawData->Az_mpss;
      pData->pitchRate_rps = -pRawData->Gr_rps;
      pData->rollRate_rps = -pRawData->Gp_rps;
      pData->yawRate_rps = -pRawData->Gy_rps;
    break;
    case IMU_MOUNTING_POSITION_HORIZONTAL_DOWNWARD_BACK:  //Add 14 (23)
      pData->latAcc_mpss = pRawData->Ax_mpss;
      pData->longAcc_mpss = -pRawData->Ay_mpss;
      pData->vertAcc_mpss = -pRawData->Az_mpss;
      pData->pitchRate_rps = pRawData->Gp_rps;
      pData->rollRate_rps = -pRawData->Gr_rps;
      pData->yawRate_rps = -pRawData->Gy_rps;
    break;
    case IMU_MOUNTING_POSITION_HORIZONTAL_DOWNWARD_RIGHT:  //Add 16 (24)
      pData->latAcc_mpss = pRawData->Ay_mpss;
      pData->longAcc_mpss = pRawData->Ax_mpss;
      pData->vertAcc_mpss = -pRawData->Az_mpss;
      pData->pitchRate_rps = pRawData->Gr_rps;
      pData->rollRate_rps = pRawData->Gp_rps;
      pData->yawRate_rps = -pRawData->Gy_rps;
    break;
    case IMU_MOUNTING_POSITION_CUSTOM:
      _applyCustomRotation(pRawData, pData);
    break;
    default:
      pData->latAcc_mpss = pRawData->Ax_mpss;
      pData->longAcc_mpss = pRawData->Ay_mpss;
      pData->vertAcc_mpss = pRawData->Az_mpss;
      pData->pitchRate_rps = pRawData->Gp_rps;
      pData->rollRate_rps = pRawData->Gr_rps;
      pData->yawRate_rps = pRawData->Gy_rps;
    break;
  }
}

void _applyCustomRotation(imu_raw_data_struct_t* pRawData, imu_data_struct_t* pData)
{
  if (imu_custom_rot_valid) {
    pData->latAcc_mpss = imu_rot_matrix[0][0] * pRawData->Ax_mpss + imu_rot_matrix[0][1] * pRawData->Ay_mpss + imu_rot_matrix[0][2] * pRawData->Az_mpss;
    pData->longAcc_mpss = imu_rot_matrix[1][0] * pRawData->Ax_mpss + imu_rot_matrix[1][1] * pRawData->Ay_mpss + imu_rot_matrix[1][2] * pRawData->Az_mpss;
    pData->vertAcc_mpss = imu_rot_matrix[2][0] * pRawData->Ax_mpss + imu_rot_matrix[2][1] * pRawData->Ay_mpss + imu_rot_matrix[2][2] * pRawData->Az_mpss;
    
    pData->pitchRate_rps = imu_rot_matrix[0][0] * pRawData->Gp_rps + imu_rot_matrix[0][1] * pRawData->Gr_rps + imu_rot_matrix[0][2] * pRawData->Gy_rps;
    pData->rollRate_rps = imu_rot_matrix[1][0] * pRawData->Gp_rps + imu_rot_matrix[1][1] * pRawData->Gr_rps + imu_rot_matrix[1][2] * pRawData->Gy_rps;
    pData->yawRate_rps = imu_rot_matrix[2][0] * pRawData->Gp_rps + imu_rot_matrix[2][1] * pRawData->Gr_rps + imu_rot_matrix[2][2] * pRawData->Gy_rps;
    
//    pData->latAcc_mpss = imu_rot_matrix[0][0] * pRawData->Ax_mpss + imu_rot_matrix[1][0] * pRawData->Ay_mpss + imu_rot_matrix[2][0] * pRawData->Az_mpss;
//    pData->longAcc_mpss = imu_rot_matrix[0][1] * pRawData->Ax_mpss + imu_rot_matrix[1][1] * pRawData->Ay_mpss + imu_rot_matrix[2][1] * pRawData->Az_mpss;
//    pData->vertAcc_mpss = imu_rot_matrix[0][2] * pRawData->Ax_mpss + imu_rot_matrix[1][2] * pRawData->Ay_mpss + imu_rot_matrix[2][2] * pRawData->Az_mpss;
//    
//    pData->pitchRate_rps = imu_rot_matrix[0][0] * pRawData->Gp_rps + imu_rot_matrix[1][0] * pRawData->Gr_rps + imu_rot_matrix[2][0] * pRawData->Gy_rps;
//    pData->rollRate_rps = imu_rot_matrix[0][1] * pRawData->Gp_rps + imu_rot_matrix[1][1] * pRawData->Gr_rps + imu_rot_matrix[2][1] * pRawData->Gy_rps;
//    pData->yawRate_rps = imu_rot_matrix[0][2] * pRawData->Gp_rps + imu_rot_matrix[1][2] * pRawData->Gr_rps + imu_rot_matrix[2][2] * pRawData->Gy_rps;
  } else {
      pData->latAcc_mpss = pRawData->Ax_mpss;
      pData->longAcc_mpss = pRawData->Ay_mpss;
      pData->vertAcc_mpss = pRawData->Az_mpss;
      pData->pitchRate_rps = pRawData->Gp_rps;
      pData->rollRate_rps = pRawData->Gr_rps;
      pData->yawRate_rps = pRawData->Gy_rps;
  }
}

bool _quat2rotM(imu_quaternion_struct_t* quat, real32_T rotM[][3])
{
  const real32_T sqEps = (1e-6);
  real32_T sqw;
  real32_T sqx;
  real32_T sqy;
  real32_T sqz;
  real32_T sqmod;
  real32_T invs;
  real32_T tmp1;
  real32_T tmp2;

  sqw = quat->qW*quat->qW;
  sqx = quat->qX*quat->qX;
  sqy = quat->qY*quat->qY;
  sqz = quat->qZ*quat->qZ;
  // invs (inverse square length) is only required if quaternion is not already normalised
  sqmod = (sqx + sqy + sqz + sqw);
  if (fabs(sqmod - 1.0f) > sqEps)
    return false;
  
  invs = 1 / sqmod;
  rotM[0][0] = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
  rotM[1][1] = (-sqx + sqy - sqz + sqw)*invs ;
  rotM[2][2] = (-sqx - sqy + sqz + sqw)*invs ;
  
  tmp1 = quat->qX*quat->qY;
  tmp2 = quat->qZ*quat->qW;
  rotM[1][0] = 2.0f * (tmp1 - tmp2)*invs ;
  rotM[0][1] = 2.0f * (tmp1 + tmp2)*invs ;
  
  tmp1 = quat->qX*quat->qZ;
  tmp2 = quat->qY*quat->qW;
  rotM[2][0] = 2.0f * (tmp1 + tmp2)*invs ;
  rotM[0][2] = 2.0f * (tmp1 - tmp2)*invs ;
  tmp1 = quat->qY*quat->qZ;
  tmp2 = quat->qX*quat->qW;
  rotM[2][1] = 2.0f * (tmp1 - tmp2)*invs ; 
  rotM[1][2] = 2.0f * (tmp1 + tmp2)*invs ;

  return true;
}

bool _proccesNewQuat(void)
{
  imu_custom_rot_valid = _quat2rotM(&imu_quaternion, imu_rot_matrix);
  _Reset_Filter();
  
  return imu_custom_rot_valid;
}

void _Reset_Filter(void)
{
    IMU_FILTER_Reset();
    IMU_CANMSG_Reset();
    imu_filer_isReset = true;
}

/* Public functions ---------------------------------------------------------*/
imu_op_status_e IMU_MANAGER_Init(imu_hal_struct_t *iHal, imu_hw_id_e iHWID)
{
  imu_op_status_e status;
  
  if (initialized)
      return IMU_STATUS_OK;
  
  imu_hal.pIMURead = iHal->pIMURead;
  imu_hal.pIMUWrite = iHal->pIMUWrite;
  imu_hal.pEEPROMRead = iHal->pEEPROMRead;
  imu_hal.pEEPROMWrite = iHal->pEEPROMWrite;
  imu_hal.pSleepMs = iHal->pSleepMs;
    
  if (imu_hal.pEEPROMRead(0, (uint8_t*) &imu_calib_data, IMU_PARAMETER_VERSION_1_TOTAL_SIZE) != IMU_HAL_STATUS_OK)
      return IMU_STATUS_ERROR_EEPROM;
    
  if (!_checkEEPROM((uint8_t*) &imu_calib_data)) {
    // Set to default
    _setDefaultCalib(&imu_calib_data);
  }

  if (imu_calib_data.imu_parameter_version == IMU_PARAMETER_VERSION_1) {
    if (imu_calib_data.imu_parameter_default_flag == 0) {
      acc_calibration_good = true;
      gyr_calibration_good = true;
    }
  } else {
    if (!(imu_calib_data.imu_parameter_default_flag & ACC_DEFAULT_FLAG)) {
        acc_calibration_good = true;
    }
    if (!(imu_calib_data.imu_parameter_default_flag & GYR_DEFAULT_FLAG)) {
        gyr_calibration_good = true;
    }
  }

#if defined(SYSTEM_STEER)

  if (iHWID == IMU_HW_ID_LSM6DSM) {
    imu_driver.pInit = &lsm6dsm_init;
    imu_driver.pProcessor = &lsm6dsm_processor;
    imu_driver.pDiagnostic = &lsm6dsm_get_diagnostic; 
    imu_driver.pGetRawValues = &lsm6dsm_get_imu_raw_values; 
    imu_driver.pGetTemperature = &lsm6dsm_get_imu_temperature;
    imu_driver.pGetTemperatures = &lsm6dsm_get_imu_temperatures;
  } else if (iHWID == IMU_HW_ID_ADC8344_ADXRS614_ADCL203) {
    imu_driver.pInit = &imu_adc8344_init;
    imu_driver.pProcessor = &imu_adc8344_processor;
    imu_driver.pDiagnostic = &imu_adc8344_get_diagnostic; 
    imu_driver.pGetRawValues = &imu_adc8344_get_imu_raw_values; 
    imu_driver.pGetTemperature = &imu_adc8344_get_imu_temperature;
    imu_driver.pGetTemperatures = &imu_adc8344_get_imu_temperatures;
  } else if (iHWID == IMU_HW_ID_ASM330LHH) {
    imu_driver.pInit = &asm330lhh_init;
    imu_driver.pProcessor = &asm330lhh_processor;
    imu_driver.pDiagnostic = &asm330lhh_get_diagnostic; 
    imu_driver.pGetRawValues = &asm330lhh_get_imu_raw_values; 
    imu_driver.pGetTemperature = &asm330lhh_get_imu_temperature;
    imu_driver.pGetTemperatures = &asm330lhh_get_imu_temperatures;
  } else {
    return IMU_STATUS_ERROR_BAD_DEVICE_ID;  
  }
  
#elif defined(SYSTEM_ECUROW) || defined(SYSTEM_SMARTANTENNA)
  
  if (iHWID != IMU_HW_ID_LSM6DSM)
    return IMU_STATUS_ERROR_BAD_DEVICE_ID;  
  
  imu_driver.pInit = &lsm6dsm_init;
  imu_driver.pProcessor = &lsm6dsm_processor;
  imu_driver.pDiagnostic = &lsm6dsm_get_diagnostic; 
  imu_driver.pGetRawValues = &lsm6dsm_get_imu_raw_values; 
  imu_driver.pGetTemperature = &lsm6dsm_get_imu_temperature; 
  imu_driver.pGetTemperatures = &lsm6dsm_get_imu_temperatures;
  
#elif defined(SYSTEM_SBOX7)
  
  if (iHWID == IMU_HW_ID_LSM6DSM) {
    imu_driver.pInit = &lsm6dsm_init;
    imu_driver.pProcessor = &lsm6dsm_processor;
    imu_driver.pDiagnostic = &lsm6dsm_get_diagnostic; 
    imu_driver.pGetRawValues = &lsm6dsm_get_imu_raw_values; 
    imu_driver.pGetTemperature = &lsm6dsm_get_imu_temperature;
    imu_driver.pGetTemperatures = &lsm6dsm_get_imu_temperatures;
  } else if (iHWID == IMU_HW_ID_ASM330LHH) {
    imu_driver.pInit = &asm330lhh_init;
    imu_driver.pProcessor = &asm330lhh_processor;
    imu_driver.pDiagnostic = &asm330lhh_get_diagnostic; 
    imu_driver.pGetRawValues = &asm330lhh_get_imu_raw_values; 
    imu_driver.pGetTemperature = &asm330lhh_get_imu_temperature;
    imu_driver.pGetTemperatures = &asm330lhh_get_imu_temperatures;
  } else {
    return IMU_STATUS_ERROR_BAD_DEVICE_ID;  
  }
  
#else
  
  return IMU_STATUS_ERROR_BAD_INIT;
  
#endif
  
  status = imu_driver.pInit(&imu_hal);
      
  if (status != IMU_STATUS_OK) {
      return IMU_STATUS_ERROR_BAD_INIT;
  }
  
  status = imu_driver.pProcessor();
  
  if (status != IMU_STATUS_OK) {
      return IMU_STATUS_ERROR_BAD_INIT;
  }
  
  hw_id = iHWID;
  initialized = true;
    
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_Processor(void)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  return imu_driver.pProcessor();
}

bool IMU_MANAGER_IsInitialized(void)
{
  return initialized;
}

bool IMU_MANAGER_IsReset(void)
{
    bool aux = imu_filer_isReset;
    
    imu_filer_isReset = false;
    
    return aux;
}    

uint32_t IMU_MANAGER_GetIMUEEPROMSize(void)
{
  return IMU_PARAMETER_VERSION_1_TOTAL_SIZE;
}

bool IMU_MANAGER_IsAccCalibrationValid(void)
{
  return acc_calibration_good;
}

bool IMU_MANAGER_IsGyrCalibrationValid(void)
{
  return gyr_calibration_good;
}

imu_op_status_e IMU_MANAGER_GetHWID(imu_hw_id_e* oHWID)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  *oHWID = hw_id;
  
  return IMU_STATUS_OK;
}

//imu_op_status_e IMU_MANAGER_GetHWSerialNumber(uint32_t* oHWSerialNumber)
//{
//  
//}

//imu_op_status_e IMU_MANAGER_GetTimestamp(uint32_t* oTimestamp)
//{
//  
//}

imu_op_status_e IMU_MANAGER_GetDiagnostic(uint32_t* oDiag)
{  
  uint32_t diag = 0x0;
  imu_op_status_e ret;
  
  if (!initialized) {
      diag |= IMU_DIAGNOSTIC_NO_INIT;
      ret = IMU_STATUS_ERROR_NO_INIT;
  } else {  
      ret = imu_driver.pDiagnostic(&diag);
      
      if (!IMU_MANAGER_IsAccCalibrationValid() || !IMU_MANAGER_IsGyrCalibrationValid())
          diag |= IMU_DIAGNOSTIC_CALIB_ERROR;
  }
  
  *oDiag = diag;
  return ret;
}

imu_op_status_e IMU_MANAGER_GetIMURawValues(imu_raw_data_struct_t* oRawValues)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  return imu_driver.pGetRawValues(oRawValues);
}

imu_op_status_e IMU_MANAGER_GetIMURawAx(real32_T* oValue)
{
  imu_op_status_e status;
  imu_raw_data_struct_t data;
  
  status = IMU_MANAGER_GetIMURawValues(&data);
  
  *oValue = data.Ax_mpss;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMURawAy(real32_T* oValue)
{
  imu_op_status_e status;
  imu_raw_data_struct_t data;
  
  status = IMU_MANAGER_GetIMURawValues(&data);
  
  *oValue = data.Ay_mpss;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMURawAz(real32_T* oValue)
{
  imu_op_status_e status;
  imu_raw_data_struct_t data;
  
  status = IMU_MANAGER_GetIMURawValues(&data);
  
  *oValue = data.Az_mpss;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMURawGp(real32_T* oValue)
{
  imu_op_status_e status;
  imu_raw_data_struct_t data;
  
  status = IMU_MANAGER_GetIMURawValues(&data);
  
  *oValue = data.Gp_rps;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMURawGr(real32_T* oValue)
{
  imu_op_status_e status;
  imu_raw_data_struct_t data;
  
  status = IMU_MANAGER_GetIMURawValues(&data);
  
  *oValue = data.Gr_rps;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMURawGy(real32_T* oValue)
{
  imu_op_status_e status;
  imu_raw_data_struct_t data;
  
  status = IMU_MANAGER_GetIMURawValues(&data);
  
  *oValue = data.Gy_rps;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUValues(imu_data_struct_t* oValues)
{
  imu_op_status_e status;
  imu_raw_data_struct_t raw_data;
  imu_temperature_struct_t temperatures;
  
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  status = imu_driver.pGetRawValues(&raw_data);
  if (status != IMU_STATUS_OK) {
    return status;
  }
  
  status = imu_driver.pGetTemperatures(&temperatures);
  if (status != IMU_STATUS_OK) {
    return status;
  }
  
  _applyCalibration(&raw_data, &temperatures);
  _applyRotation(&raw_data, oValues);
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_GetIMUALat(real32_T* oValue)
{
  imu_op_status_e status;
  imu_data_struct_t data;
  
  status = IMU_MANAGER_GetIMUValues(&data);
  
  *oValue = data.latAcc_mpss;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUALong(real32_T* oValue)
{
  imu_op_status_e status;
  imu_data_struct_t data;
  
  status = IMU_MANAGER_GetIMUValues(&data);
  
  *oValue = data.longAcc_mpss;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUAVert(real32_T* oValue)
{
  imu_op_status_e status;
  imu_data_struct_t data;
  
  status = IMU_MANAGER_GetIMUValues(&data);
  
  *oValue = data.vertAcc_mpss;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUGPitchRate(real32_T* oValue)
{
  imu_op_status_e status;
  imu_data_struct_t data;
  
  status = IMU_MANAGER_GetIMUValues(&data);
  
  *oValue = data.pitchRate_rps;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUGRollRate(real32_T* oValue)
{
  imu_op_status_e status;
  imu_data_struct_t data;
  
  status = IMU_MANAGER_GetIMUValues(&data);
  
  *oValue = data.rollRate_rps;
  
  return status;  
}


imu_op_status_e IMU_MANAGER_GetIMUGYawRate(real32_T* oValue)
{
  imu_op_status_e status;
  imu_data_struct_t data;
  
  status = IMU_MANAGER_GetIMUValues(&data);
  
  *oValue = data.yawRate_rps;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUTemperature(real32_T* oTemperature)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  return imu_driver.pGetTemperature(oTemperature);
}

imu_op_status_e IMU_MANAGER_GetIMUTemperatures(imu_temperature_struct_t* oTemperatures)
{
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;
  
  return imu_driver.pGetTemperatures(oTemperatures);
}

imu_op_status_e IMU_MANAGER_GetIMUTempAx(real32_T* oValue)
{
  imu_op_status_e status;
  imu_temperature_struct_t data;
  
  status = IMU_MANAGER_GetIMUTemperatures(&data);
  
  *oValue = data.Ax_Temp_degC;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUTempAy(real32_T* oValue)
{
  imu_op_status_e status;
  imu_temperature_struct_t data;
  
  status = IMU_MANAGER_GetIMUTemperatures(&data);
  
  *oValue = data.Ay_Temp_degC;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUTempAz(real32_T* oValue)
{
  imu_op_status_e status;
  imu_temperature_struct_t data;
  
  status = IMU_MANAGER_GetIMUTemperatures(&data);
  
  *oValue = data.Az_Temp_degC;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUTempGp(real32_T* oValue)
{
  imu_op_status_e status;
  imu_temperature_struct_t data;
  
  status = IMU_MANAGER_GetIMUTemperatures(&data);
  
  *oValue = data.Gp_Temp_degC;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUTempGr(real32_T* oValue)
{
  imu_op_status_e status;
  imu_temperature_struct_t data;
  
  status = IMU_MANAGER_GetIMUTemperatures(&data);
  
  *oValue = data.Gr_Temp_degC;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_GetIMUTempGy(real32_T* oValue)
{
  imu_op_status_e status;
  imu_temperature_struct_t data;
  
  status = IMU_MANAGER_GetIMUTemperatures(&data);
  
  *oValue = data.Gy_Temp_degC;
  
  return status;  
}

imu_op_status_e IMU_MANAGER_SetCalibrationParameter(imu_calibration_parameters_e id, real32_T value)
{
  real32_T* pParam;
  
  pParam = _getParamPtr(id);
  
  if (pParam == NULL) {
    return IMU_STATUS_ERROR_BAD_PARAM_ID;
  } else {
    *pParam = value;
    // Config is no longer default
    if (_isAccParameter(id)) {
      imu_calib_data.imu_parameter_default_flag &= ~ACC_DEFAULT_FLAG;
    } else if (_isGyrParameter(id)) {
      imu_calib_data.imu_parameter_default_flag &= ~GYR_DEFAULT_FLAG;
    }
    return IMU_STATUS_OK;    
  }
}

imu_op_status_e IMU_MANAGER_GetCalibrationParameter(imu_calibration_parameters_e id, real32_T *value)
{
  real32_T* pParam;
  
  pParam = _getParamPtr(id);
  
  if (pParam == NULL) {
    return IMU_STATUS_ERROR_BAD_PARAM_ID;
  } else {
    *value = *pParam;
    return IMU_STATUS_OK;    
  }
}

imu_op_status_e IMU_MANAGER_SetMountingPosition(uint8_t value)
{
  if (value >= IMU_MOUNTING_POSITION_LAST)
    return IMU_STATUS_ERROR_BAD_CONFIGURATION;  
  
  imu_mounting_position = (imu_mounting_positions_e) value;
  _Reset_Filter();
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_GetMountingPosition(uint8_t *value)
{
  *value = (uint8_t) imu_mounting_position;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_SetMountingQuaternion(imu_quaternion_struct_t value)
{
  imu_quaternion.qW = value.qW;
  imu_quaternion.qX = value.qX;
  imu_quaternion.qY = value.qY;
  imu_quaternion.qZ = value.qZ;
  
  if(_proccesNewQuat()) {
    return IMU_STATUS_OK;
  } else {
    return IMU_STATUS_ERROR_BAD_CONFIGURATION;
  }
}

imu_op_status_e IMU_MANAGER_SetMountingQuaternionQw(real32_T value)
{
  imu_quaternion.qW = value;
  imu_custom_rot_valid = false;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_SetMountingQuaternionQx(real32_T value)
{
  imu_quaternion.qX = value;
  imu_custom_rot_valid = false;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_SetMountingQuaternionQy(real32_T value)
{
  imu_quaternion.qY = value;
  imu_custom_rot_valid = false;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_SetMountingQuaternionQz(real32_T value)
{
  imu_quaternion.qZ = value;
  imu_custom_rot_valid = false;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_SetMountingQuaternionEnd(void)
{
  if(_proccesNewQuat()) {
    return IMU_STATUS_OK;
  } else {
    return IMU_STATUS_ERROR_BAD_CONFIGURATION;
  }
}

imu_op_status_e IMU_MANAGER_GetMountingQuaternion(imu_quaternion_struct_t *value)
{
  value->qW = imu_quaternion.qW;
  value->qX = imu_quaternion.qX;
  value->qY = imu_quaternion.qY;
  value->qZ = imu_quaternion.qZ;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_GetMountingQuaternionQw(real32_T *value)
{
  *value = imu_quaternion.qW;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_GetMountingQuaternionQx(real32_T *value)
{
  *value = imu_quaternion.qX;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_GetMountingQuaternionQy(real32_T *value)
{
  *value = imu_quaternion.qY;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_GetMountingQuaternionQz(real32_T *value)
{
  *value = imu_quaternion.qZ;
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_RestoreDefaultConfigurationAcc(void)
{
  _setDefaultAccCalib(&imu_calib_data);
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_RestoreDefaultConfigurationGyr(void)
{
  _setDefaultGyrCalib(&imu_calib_data);
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_RestoreDefaultConfigurationFull(void)
{
  _setDefaultCalib(&imu_calib_data);
  
  return IMU_STATUS_OK;
}

imu_op_status_e IMU_MANAGER_PersistConfiguration(void)
{
  uint8_t checksum;
  bool tmp;
  
  _calcChecksum((uint8_t*)&imu_calib_data, &checksum, &tmp);
  imu_calib_data.checksum = checksum;
  
  return imu_hal.pEEPROMWrite(0, (uint8_t*)&imu_calib_data, IMU_PARAMETER_VERSION_1_TOTAL_SIZE) == IMU_HAL_STATUS_OK? IMU_STATUS_OK : IMU_STATUS_ERROR_EEPROM;
}
