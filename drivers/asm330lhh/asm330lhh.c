
/* Includes ------------------------------------------------------------------*/
#include "drivers\asm330lhh\asm330lhh.h"

/* Statics ********************************************************************/
static bool initialized = false;
static bool setup = false;
static imu_hal_struct_t *imu_hal;
static asm330lhh_config_struct_t asm330lhh_config;
static asm330lhh_data_struct_t asm330lhh_data;
static uint32_t diagnostic = 0x0;

/* Defines ********************************************************************/
#define MIN_TEMPERATURE_DEG (-40.0f)
#define MAX_TEMPERATURE_DEG (105.0f)

#define SETUP_RETRY_TIME (1000U);
/* Private functions ---------------------------------------------------------*/
static void xl_fs_to_mg(axis3bit16_t raw_xl, float xl[], asm330lhh_fs_xl_t xl_fs)
{
    switch(xl_fs) {
        case ASM330LHH_2g:
          xl[0] = asm330lhh_from_fs2g_to_mg(raw_xl.i16bit[0]);
          xl[1] = asm330lhh_from_fs2g_to_mg(raw_xl.i16bit[1]);
          xl[2] = asm330lhh_from_fs2g_to_mg(raw_xl.i16bit[2]);
        break;
        case ASM330LHH_4g:
          xl[0] = asm330lhh_from_fs4g_to_mg(raw_xl.i16bit[0]);
          xl[1] = asm330lhh_from_fs4g_to_mg(raw_xl.i16bit[1]);
          xl[2] = asm330lhh_from_fs4g_to_mg(raw_xl.i16bit[2]);
        break;
        case ASM330LHH_8g:
          xl[0] = asm330lhh_from_fs8g_to_mg(raw_xl.i16bit[0]);
          xl[1] = asm330lhh_from_fs8g_to_mg(raw_xl.i16bit[1]);
          xl[2] = asm330lhh_from_fs8g_to_mg(raw_xl.i16bit[2]);
        break;
        case ASM330LHH_16g:
          xl[0] = asm330lhh_from_fs16g_to_mg(raw_xl.i16bit[0]);
          xl[1] = asm330lhh_from_fs16g_to_mg(raw_xl.i16bit[1]);
          xl[2] = asm330lhh_from_fs16g_to_mg(raw_xl.i16bit[2]);
        break;        
    }
}

static void gy_fs_to_mdps(axis3bit16_t raw_gy, float gy[], asm330lhh_fs_g_t g_fs)
{
    switch(g_fs) {
        case ASM330LHH_125dps:
          gy[0] = asm330lhh_from_fs125dps_to_mdps(raw_gy.i16bit[0]);
          gy[1] = asm330lhh_from_fs125dps_to_mdps(raw_gy.i16bit[1]);
          gy[2] = asm330lhh_from_fs125dps_to_mdps(raw_gy.i16bit[2]);
        break;
        case ASM330LHH_250dps:
          gy[0] = asm330lhh_from_fs250dps_to_mdps(raw_gy.i16bit[0]);
          gy[1] = asm330lhh_from_fs250dps_to_mdps(raw_gy.i16bit[1]);
          gy[2] = asm330lhh_from_fs250dps_to_mdps(raw_gy.i16bit[2]);
        break;
        case ASM330LHH_500dps:
          gy[0] = asm330lhh_from_fs500dps_to_mdps(raw_gy.i16bit[0]);
          gy[1] = asm330lhh_from_fs500dps_to_mdps(raw_gy.i16bit[1]);
          gy[2] = asm330lhh_from_fs500dps_to_mdps(raw_gy.i16bit[2]);
        break;
        case ASM330LHH_1000dps:
          gy[0] = asm330lhh_from_fs1000dps_to_mdps(raw_gy.i16bit[0]);
          gy[1] = asm330lhh_from_fs1000dps_to_mdps(raw_gy.i16bit[1]);
          gy[2] = asm330lhh_from_fs1000dps_to_mdps(raw_gy.i16bit[2]);
        break;
        case ASM330LHH_2000dps:
          gy[0] = asm330lhh_from_fs2000dps_to_mdps(raw_gy.i16bit[0]);
          gy[1] = asm330lhh_from_fs2000dps_to_mdps(raw_gy.i16bit[1]);
          gy[2] = asm330lhh_from_fs2000dps_to_mdps(raw_gy.i16bit[2]);
        break;
        case ASM330LHH_4000dps:
          gy[0] = asm330lhh_from_fs4000dps_to_mdps(raw_gy.i16bit[0]);
          gy[1] = asm330lhh_from_fs4000dps_to_mdps(raw_gy.i16bit[1]);
          gy[2] = asm330lhh_from_fs4000dps_to_mdps(raw_gy.i16bit[2]);
        break;          
    }
}

static void temp_fs_to_degc(axis1bit16_t raw_temp, float* temp)
{
    *temp = asm330lhh_from_lsb_to_celsius(raw_temp.i16bit);
}

static void _acc_orientation(float* acceleration_mg)
{
  float tmp;
#if defined(SYSTEM_STEER)
  tmp = acceleration_mg[1];
  acceleration_mg[1] = -acceleration_mg[0];
  acceleration_mg[0] = tmp;
  acceleration_mg[2] = -acceleration_mg[2];
#elif defined(SYSTEM_ECUROW)
  tmp = acceleration_mg[0];
  acceleration_mg[0] =  acceleration_mg[2];
  acceleration_mg[2] = -acceleration_mg[1];
  acceleration_mg[1] =  tmp;
#elif defined(SYSTEM_SMARTANTENNA)
  tmp = acceleration_mg[1];
  acceleration_mg[1] = -acceleration_mg[0];
  acceleration_mg[0] = tmp;
  acceleration_mg[2] = -acceleration_mg[2];
#elif defined(SYSTEM_SBOX7)
  tmp = acceleration_mg[1];
  acceleration_mg[1] = -acceleration_mg[0];
  acceleration_mg[0] = tmp;
  acceleration_mg[2] = -acceleration_mg[2];
#else
  // Leave as is
  #warning "Axis rotation not defined"
#endif
  
  return;
}

static void _gyr_orientation(float* angular_rate_mdps)
{
  float tmp;
#if defined(SYSTEM_STEER)
  tmp = angular_rate_mdps[1];
  angular_rate_mdps[1] = angular_rate_mdps[0];
  angular_rate_mdps[0] = -tmp;
  angular_rate_mdps[2] = angular_rate_mdps[2];
#elif defined(SYSTEM_ECUROW)
  tmp = angular_rate_mdps[1];
  angular_rate_mdps[1] = -angular_rate_mdps[0];
  angular_rate_mdps[0] = -angular_rate_mdps[2];
  angular_rate_mdps[2] = -tmp;
#elif defined(SYSTEM_SMARTANTENNA)
  tmp = angular_rate_mdps[1];
  angular_rate_mdps[1] = angular_rate_mdps[0];
  angular_rate_mdps[0] = -tmp;
  angular_rate_mdps[2] = angular_rate_mdps[2];
#elif defined(SYSTEM_SBOX7)
  tmp = angular_rate_mdps[1];
  angular_rate_mdps[1] = angular_rate_mdps[0];
  angular_rate_mdps[0] = -tmp;
  angular_rate_mdps[2] = angular_rate_mdps[2];
#else
  // Leave as is
  #warning "Axis rotation not defined"
#endif
  
  return;
}

static int32_t _read(void *ctx, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  if (!imu_hal->pIMURead)
    return ASM330LHH_HAL_STATUS_FAILED;

  return imu_hal->pIMURead(&ReadAddr, 1, pBuffer, nBytesToRead) == IMU_HAL_STATUS_OK? ASM330LHH_HAL_STATUS_OK : ASM330LHH_HAL_STATUS_FAILED;
}

static int32_t _write(void *ctx, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  if (!imu_hal->pIMUWrite)
    return ASM330LHH_HAL_STATUS_FAILED;

  return imu_hal->pIMUWrite(&WriteAddr, 1, pBuffer, nBytesToWrite) == IMU_HAL_STATUS_OK? ASM330LHH_HAL_STATUS_OK : ASM330LHH_HAL_STATUS_FAILED;
}

/* Public functions ---------------------------------------------------------*/
imu_op_status_e asm330lhh_init (imu_hal_struct_t *iHal)
{
  asm330lhh_ctx_t *dev_ctx = &asm330lhh_config.dev_ctx;
  imu_hal = iHal;
  
  if (initialized)
    return IMU_STATUS_OK;
  
  /*
   *  Initialize mems driver interface.
   */
  dev_ctx->write_reg = &_write;
  dev_ctx->read_reg = &_read;

  initialized = true;
  
  return IMU_STATUS_OK;
}

imu_op_status_e asm330lhh_setup (void)
{
  int32_t ret;
  uint8_t whoamI, rst;
  asm330lhh_config_mode iConfig_mode;
  
  asm330lhh_ctx_t *dev_ctx = &asm330lhh_config.dev_ctx;
    
  diagnostic = IMU_DIAGNOSTIC_COMM_ERROR;
  
  setup = false;
    
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;

  /*
   *  Check device ID.
   */
  whoamI = 0;
  ret = 1;
  if (!asm330lhh_device_id_get(dev_ctx, &whoamI)) {
    return IMU_STATUS_ERROR_COMM;
  }
  if ( whoamI != ASM330LHH_ID ) {
    return IMU_STATUS_ERROR_BAD_DEVICE_ID;
  }

  /*
   *  Restore default configuration.
   */
  if (!asm330lhh_reset_set(dev_ctx, PROPERTY_ENABLE)) {
    return IMU_STATUS_ERROR_COMM;
  }
  do {
    if (!asm330lhh_reset_get(dev_ctx, &rst)) {
        return IMU_STATUS_ERROR_COMM;
    }
  } while (rst);

#if defined(SYSTEM_STEER)
  iConfig_mode = ASM330LHH_CONFIG_MODE_PILOT;
#elif defined(SYSTEM_ECUROW)
  iConfig_mode = ASM330LHH_CONFIG_MODE_ECUROW;
#elif defined(SYSTEM_SMARTANTENNA)
  iConfig_mode = ASM330LHH_CONFIG_MODE_SMARTANTENNA;
#elif defined(SYSTEM_SBOX7)
  iConfig_mode = ASM330LHH_CONFIG_MODE_LPC43xx;
#else
  iConfig_mode = ASM330LHH_CONFIG_MODE_DEFAULT;
#endif
  
  if (iConfig_mode != ASM330LHH_CONFIG_MODE_DEFAULT) {
    switch (iConfig_mode) {
      case ASM330LHH_CONFIG_MODE_PILOT:
      case ASM330LHH_CONFIG_MODE_ECUROW:
      case ASM330LHH_CONFIG_MODE_SMARTANTENNA:
      case ASM330LHH_CONFIG_MODE_LPC43xx:
        asm330lhh_config.fifo_mode = ASM330LHH_STREAM_MODE;
        asm330lhh_config.bdu = PROPERTY_ENABLE;
        
        asm330lhh_config.xl_fs = ASM330LHH_4g;
        asm330lhh_config.xl_odr = ASM330LHH_XL_ODR_833Hz;
        asm330lhh_config.xl_hp_slope_en = ASM330LHH_LP_ODR_DIV_100;
        
        asm330lhh_config.g_fs = ASM330LHH_125dps;
        asm330lhh_config.g_odr = ASM330LHH_GY_ODR_104Hz;
        break;
      default:
          return IMU_STATUS_ERROR_BAD_CONFIGURATION;
    }
    
    ret *= asm330lhh_fifo_mode_set(dev_ctx, asm330lhh_config.fifo_mode);
    ret *= asm330lhh_block_data_update_set(dev_ctx, asm330lhh_config.bdu);
    
    ret *= asm330lhh_xl_full_scale_set(dev_ctx, asm330lhh_config.xl_fs);
    ret *= asm330lhh_xl_data_rate_set(dev_ctx, asm330lhh_config.xl_odr);
    ret *= asm330lhh_xl_hp_path_on_out_set(dev_ctx, asm330lhh_config.xl_hp_slope_en);
    
    ret *= asm330lhh_gy_full_scale_set(dev_ctx, asm330lhh_config.g_fs);
    ret *= asm330lhh_gy_data_rate_set(dev_ctx, asm330lhh_config.g_odr);
    
    if (!ret)
      return IMU_STATUS_ERROR_COMM;
  }
  
  ret *= asm330lhh_fifo_mode_get(dev_ctx, &asm330lhh_config.fifo_mode);
  ret *= asm330lhh_block_data_update_get(dev_ctx, &asm330lhh_config.bdu);
  
  ret *= asm330lhh_xl_full_scale_get(dev_ctx, &asm330lhh_config.xl_fs);
  ret *= asm330lhh_xl_data_rate_get(dev_ctx, &asm330lhh_config.xl_odr);
  ret *= asm330lhh_xl_hp_path_on_out_get(dev_ctx, &asm330lhh_config.xl_hp_slope_en);
  
  ret *= asm330lhh_gy_full_scale_get(dev_ctx, &asm330lhh_config.g_fs);
  ret *= asm330lhh_gy_data_rate_get(dev_ctx, &asm330lhh_config.g_odr);
  
  if (!ret)
    return IMU_STATUS_ERROR_COMM;
  
  diagnostic = 0x0;
  setup = true;
  
  return IMU_STATUS_OK;
}

imu_op_status_e asm330lhh_processor (void)
{
  int i;
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
  axis1bit16_t data_raw_temperature;
  
  asm330lhh_reg_t reg;
  asm330lhh_ctx_t *dev_ctx = &asm330lhh_config.dev_ctx;
    
  imu_op_status_e ret;
  
  // Update diagnostic
  diagnostic = IMU_DIAGNOSTIC_COMM_ERROR;

  /*
   * Check Init.
   */
  if (!initialized) {
    return IMU_STATUS_ERROR_NO_INIT;
  }
  
  /*
   * Check Setup.
   */
  if (!setup) {
    ret = asm330lhh_setup();
    if (ret != IMU_STATUS_OK) {
        return ret;
    }
  }
  
  /*
   * Read output only if new value is available.
   */
  if (!asm330lhh_status_reg_get(dev_ctx, &reg.status_reg)) {
    return IMU_STATUS_ERROR_COMM;
  }

  if (reg.status_reg.xlda)
  {
    /*
     * Read acceleration field data.
     */
    for (i=0; i<3; i++) {
        data_raw_acceleration.i16bit[i] = 0x0;
    }
    if (!asm330lhh_acceleration_raw_get(dev_ctx, data_raw_acceleration.u8bit)) {
      return IMU_STATUS_ERROR_COMM;
    }
    xl_fs_to_mg(data_raw_acceleration, asm330lhh_data.acceleration_mg, asm330lhh_config.xl_fs);

    // Adjust IMU orientation
    _acc_orientation(asm330lhh_data.acceleration_mg);
  } 
  if (reg.status_reg.gda)
  {
    /*
     * Read angular rate field data.
     */
    for (i=0; i<3; i++) {
        data_raw_angular_rate.i16bit[i] = 0x0;
    }
    if (!asm330lhh_angular_rate_raw_get(dev_ctx, data_raw_angular_rate.u8bit)) {
      return IMU_STATUS_ERROR_COMM;
    }
    gy_fs_to_mdps(data_raw_angular_rate, asm330lhh_data.angular_rate_mdps, asm330lhh_config.g_fs);
    
    // Adjust IMU orientation
    _gyr_orientation(asm330lhh_data.angular_rate_mdps);
  }
  if (reg.status_reg.tda)
  {   
    /*
     * Read temperature data.
     */
    data_raw_temperature.i16bit = 0x0;
    if (!asm330lhh_temperature_raw_get(dev_ctx, data_raw_temperature.u8bit)) {
      return IMU_STATUS_ERROR_COMM;
    }
    temp_fs_to_degc(data_raw_temperature, &(asm330lhh_data.temperature_degC));
  }
  
  diagnostic = 0x0;

  if ((asm330lhh_data.temperature_degC < MIN_TEMPERATURE_DEG) ||
      (asm330lhh_data.temperature_degC > MAX_TEMPERATURE_DEG)) {
        diagnostic |= IMU_DIAGNOSTIC_TEMPP_ERROR;
        diagnostic |= IMU_DIAGNOSTIC_TEMPR_ERROR;
        diagnostic |= IMU_DIAGNOSTIC_TEMPY_ERROR;
  }

  return IMU_STATUS_OK;
}

imu_op_status_e asm330lhh_get_diagnostic (uint32_t* oDiag)
{  
  *oDiag = diagnostic;
  
  return IMU_STATUS_OK;
}

imu_op_status_e asm330lhh_get_imu_raw_values (imu_raw_data_struct_t* oRawValues)
{  
  oRawValues->Ax_mpss = asm330lhh_data.acceleration_mg[0] / 1000.0f * ECUATORIAL_EFFECTIVE_GRAVITY;
  oRawValues->Ay_mpss = asm330lhh_data.acceleration_mg[1] / 1000.0f * ECUATORIAL_EFFECTIVE_GRAVITY;
  oRawValues->Az_mpss = asm330lhh_data.acceleration_mg[2] / 1000.0f * ECUATORIAL_EFFECTIVE_GRAVITY;
  oRawValues->Gp_rps = asm330lhh_data.angular_rate_mdps[0] / 1000.0f * DEG_TO_RAD;
  oRawValues->Gr_rps = asm330lhh_data.angular_rate_mdps[1] / 1000.0f * DEG_TO_RAD;
  oRawValues->Gy_rps = asm330lhh_data.angular_rate_mdps[2] / 1000.0f * DEG_TO_RAD;
  
  return IMU_STATUS_OK;
}

imu_op_status_e asm330lhh_get_imu_temperature (real32_T* oTemperature)
{
  *oTemperature = asm330lhh_data.temperature_degC;
  
  return IMU_STATUS_OK;
}

imu_op_status_e asm330lhh_get_imu_temperatures (imu_temperature_struct_t* oTemperatures)
{
  oTemperatures->Ax_Temp_degC = asm330lhh_data.temperature_degC;
  oTemperatures->Ay_Temp_degC = asm330lhh_data.temperature_degC;
  oTemperatures->Az_Temp_degC = asm330lhh_data.temperature_degC;
  oTemperatures->Gp_Temp_degC = asm330lhh_data.temperature_degC;
  oTemperatures->Gr_Temp_degC = asm330lhh_data.temperature_degC;
  oTemperatures->Gy_Temp_degC = asm330lhh_data.temperature_degC;
  
  return IMU_STATUS_OK;
}

imu_op_status_e asm330lhh_get_config(asm330lhh_config_struct_t **oConfig)
{
  *oConfig = &asm330lhh_config;

  return IMU_STATUS_OK;
}
