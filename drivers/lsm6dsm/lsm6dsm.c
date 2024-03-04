
/* Includes ------------------------------------------------------------------*/
#include "drivers\lsm6dsm\lsm6dsm.h"

/* Statics ********************************************************************/
static bool initialized = false;
static bool setup = false;
static imu_hal_struct_t *imu_hal;
static lsm6dsm_config_struct_t lsm6dsm_config;
static lsm6dsm_data_struct_t lsm6dsm_data;
static uint32_t diagnostic = 0x0;

/* Defines ********************************************************************/
#define MIN_TEMPERATURE_DEG (-40.0f)
#define MAX_TEMPERATURE_DEG (85.0f)

#define SETUP_RETRY_TIME (1000U);
/* Private functions ---------------------------------------------------------*/
static void xl_fs_to_mg(axis3bit16_t raw_xl, float xl[], lsm6dsm_fs_xl_t xl_fs)
{
    switch(xl_fs) {
        case LSM6DSM_2g:
          xl[0] = LSM6DSM_FROM_FS_2g_TO_mg(raw_xl.i16bit[0]);
          xl[1] = LSM6DSM_FROM_FS_2g_TO_mg(raw_xl.i16bit[1]);
          xl[2] = LSM6DSM_FROM_FS_2g_TO_mg(raw_xl.i16bit[2]);
        break;
        case LSM6DSM_4g:
          xl[0] = LSM6DSM_FROM_FS_4g_TO_mg(raw_xl.i16bit[0]);
          xl[1] = LSM6DSM_FROM_FS_4g_TO_mg(raw_xl.i16bit[1]);
          xl[2] = LSM6DSM_FROM_FS_4g_TO_mg(raw_xl.i16bit[2]);
        break;
        case LSM6DSM_8g:
          xl[0] = LSM6DSM_FROM_FS_8g_TO_mg(raw_xl.i16bit[0]);
          xl[1] = LSM6DSM_FROM_FS_8g_TO_mg(raw_xl.i16bit[1]);
          xl[2] = LSM6DSM_FROM_FS_8g_TO_mg(raw_xl.i16bit[2]);
        break;
        case LSM6DSM_16g:
          xl[0] = LSM6DSM_FROM_FS_16g_TO_mg(raw_xl.i16bit[0]);
          xl[1] = LSM6DSM_FROM_FS_16g_TO_mg(raw_xl.i16bit[1]);
          xl[2] = LSM6DSM_FROM_FS_16g_TO_mg(raw_xl.i16bit[2]);
        break;        
    }
}

static void gy_fs_to_mdps(axis3bit16_t raw_gy, float gy[], lsm6dsm_fs_g_t g_fs)
{
    switch(g_fs) {
        case LSM6DSM_125dps:
          gy[0] = LSM6DSM_FROM_FS_125dps_TO_mdps(raw_gy.i16bit[0]);
          gy[1] = LSM6DSM_FROM_FS_125dps_TO_mdps(raw_gy.i16bit[1]);
          gy[2] = LSM6DSM_FROM_FS_125dps_TO_mdps(raw_gy.i16bit[2]);
        break;
        case LSM6DSM_250dps:
          gy[0] = LSM6DSM_FROM_FS_250dps_TO_mdps(raw_gy.i16bit[0]);
          gy[1] = LSM6DSM_FROM_FS_250dps_TO_mdps(raw_gy.i16bit[1]);
          gy[2] = LSM6DSM_FROM_FS_250dps_TO_mdps(raw_gy.i16bit[2]);
        break;
        case LSM6DSM_500dps:
          gy[0] = LSM6DSM_FROM_FS_500dps_TO_mdps(raw_gy.i16bit[0]);
          gy[1] = LSM6DSM_FROM_FS_500dps_TO_mdps(raw_gy.i16bit[1]);
          gy[2] = LSM6DSM_FROM_FS_500dps_TO_mdps(raw_gy.i16bit[2]);
        break;
        case LSM6DSM_1000dps:
          gy[0] = LSM6DSM_FROM_FS_1000dps_TO_mdps(raw_gy.i16bit[0]);
          gy[1] = LSM6DSM_FROM_FS_1000dps_TO_mdps(raw_gy.i16bit[1]);
          gy[2] = LSM6DSM_FROM_FS_1000dps_TO_mdps(raw_gy.i16bit[2]);
        break;
        case LSM6DSM_2000dps:
          gy[0] = LSM6DSM_FROM_FS_2000dps_TO_mdps(raw_gy.i16bit[0]);
          gy[1] = LSM6DSM_FROM_FS_2000dps_TO_mdps(raw_gy.i16bit[1]);
          gy[2] = LSM6DSM_FROM_FS_2000dps_TO_mdps(raw_gy.i16bit[2]);
        break;          
    }
}

static void temp_fs_to_degc(axis1bit16_t raw_temp, float* temp)
{
    *temp = (((float)(raw_temp.i16bit) / 256.0f) + 25.0f);
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
    return LSM6DSM_HAL_STATUS_FAILED;

  return imu_hal->pIMURead(&ReadAddr, 1, pBuffer, nBytesToRead) == IMU_HAL_STATUS_OK? LSM6DSM_HAL_STATUS_OK : LSM6DSM_HAL_STATUS_FAILED;
}

static int32_t _write(void *ctx, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  if (!imu_hal->pIMUWrite)
    return LSM6DSM_HAL_STATUS_FAILED;

  return imu_hal->pIMUWrite(&WriteAddr, 1, pBuffer, nBytesToWrite) == IMU_HAL_STATUS_OK? LSM6DSM_HAL_STATUS_OK : LSM6DSM_HAL_STATUS_FAILED;
}

/* Public functions ---------------------------------------------------------*/
imu_op_status_e lsm6dsm_init (imu_hal_struct_t *iHal)
{
  lsm6dsm_ctx_t *dev_ctx = &lsm6dsm_config.dev_ctx;
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

imu_op_status_e lsm6dsm_setup (void)
{
  int32_t ret;
  uint8_t whoamI, rst;
  lsm6dsm_config_mode iConfig_mode;
  
  lsm6dsm_ctx_t *dev_ctx = &lsm6dsm_config.dev_ctx;
    
  diagnostic = IMU_DIAGNOSTIC_COMM_ERROR;
  
  setup = false;
    
  if (!initialized)
    return IMU_STATUS_ERROR_NO_INIT;

  /*
   *  Check device ID.
   */
  whoamI = 0;
  ret = 1;
  if (!lsm6dsm_device_id_get(dev_ctx, &whoamI)) {
    return IMU_STATUS_ERROR_COMM;
  }
  if ( whoamI != LSM6DSM_ID ) {
    return IMU_STATUS_ERROR_BAD_DEVICE_ID;
  }

  /*
   *  Restore default configuration.
   */
  if (!lsm6dsm_reset_set(dev_ctx, PROPERTY_ENABLE)) {
    return IMU_STATUS_ERROR_COMM;
  }
  do {
    if (!lsm6dsm_reset_get(dev_ctx, &rst)) {
        return IMU_STATUS_ERROR_COMM;
    }
  } while (rst);

#if defined(SYSTEM_STEER)
  iConfig_mode = LSM6DSM_CONFIG_MODE_PILOT;
#elif defined(SYSTEM_ECUROW)
  iConfig_mode = LSM6DSM_CONFIG_MODE_ECUROW;
#elif defined(SYSTEM_SMARTANTENNA)
  iConfig_mode = LSM6DSM_CONFIG_MODE_SMARTANTENNA;
#elif defined(SYSTEM_SBOX7)
  iConfig_mode = LSM6DSM_CONFIG_MODE_LPC43xx;
#else
  iConfig_mode = LSM6DSM_CONFIG_MODE_DEFAULT;
#endif
  
  if (iConfig_mode != LSM6DSM_CONFIG_MODE_DEFAULT) {
    switch (iConfig_mode) {
      case LSM6DSM_CONFIG_MODE_PILOT:
      case LSM6DSM_CONFIG_MODE_ECUROW:
      case LSM6DSM_CONFIG_MODE_SMARTANTENNA:
      case LSM6DSM_CONFIG_MODE_LPC43xx:
        lsm6dsm_config.fifo_mode = LSM6DSM_STREAM_MODE;
        lsm6dsm_config.bdu = PROPERTY_ENABLE;
        
        lsm6dsm_config.xl_hm_mode = LSM6DSM_XL_HIGH_PERFORMANCE;
        lsm6dsm_config.xl_fs = LSM6DSM_4g;
        lsm6dsm_config.xl_odr = LSM6DSM_XL_ODR_833Hz;
        lsm6dsm_config.xl_fifo_dec = LSM6DSM_FIFO_XL_DISABLE;
        lsm6dsm_config.xl_input_composite = LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_100;
        
        lsm6dsm_config.g_hm_mode = LSM6DSM_GY_HIGH_PERFORMANCE;
        lsm6dsm_config.g_fs = LSM6DSM_125dps;
        lsm6dsm_config.g_odr = LSM6DSM_GY_ODR_104Hz;
        lsm6dsm_config.g_fifo_dec = LSM6DSM_FIFO_GY_DISABLE;
        break;
      default:
          return IMU_STATUS_ERROR_BAD_CONFIGURATION;
    }
    
    ret *= lsm6dsm_fifo_mode_set(dev_ctx, lsm6dsm_config.fifo_mode);
    ret *= lsm6dsm_block_data_update_set(dev_ctx, lsm6dsm_config.bdu);
    
    ret *= lsm6dsm_xl_power_mode_set(dev_ctx, lsm6dsm_config.xl_hm_mode);
    ret *= lsm6dsm_xl_full_scale_set(dev_ctx, lsm6dsm_config.xl_fs);
    ret *= lsm6dsm_xl_data_rate_set(dev_ctx, lsm6dsm_config.xl_odr);
    ret *= lsm6dsm_fifo_xl_batch_set(dev_ctx, lsm6dsm_config.xl_fifo_dec);
    ret *= lsm6dsm_xl_lp2_bandwidth_set(dev_ctx, lsm6dsm_config.xl_input_composite);
    
    ret *= lsm6dsm_gy_power_mode_set(dev_ctx, lsm6dsm_config.g_hm_mode);
    ret *= lsm6dsm_gy_full_scale_set(dev_ctx, lsm6dsm_config.g_fs);
    ret *= lsm6dsm_gy_data_rate_set(dev_ctx, lsm6dsm_config.g_odr);
    ret *= lsm6dsm_fifo_gy_batch_set(dev_ctx, lsm6dsm_config.g_fifo_dec);  
    
    if (!ret)
      return IMU_STATUS_ERROR_COMM;
  }
  
  ret *= lsm6dsm_fifo_mode_get(dev_ctx, &lsm6dsm_config.fifo_mode);
  ret *= lsm6dsm_block_data_update_get(dev_ctx, &lsm6dsm_config.bdu);
  
  ret *= lsm6dsm_xl_power_mode_get(dev_ctx, &lsm6dsm_config.xl_hm_mode);
  ret *= lsm6dsm_xl_full_scale_get(dev_ctx, &lsm6dsm_config.xl_fs);
  ret *= lsm6dsm_xl_data_rate_get(dev_ctx, &lsm6dsm_config.xl_odr);
  ret *= lsm6dsm_fifo_xl_batch_get(dev_ctx, &lsm6dsm_config.xl_fifo_dec);
  ret *= lsm6dsm_xl_lp2_bandwidth_get(dev_ctx, &lsm6dsm_config.xl_input_composite);
  
  ret *= lsm6dsm_gy_power_mode_get(dev_ctx, &lsm6dsm_config.g_hm_mode);
  ret *= lsm6dsm_gy_full_scale_get(dev_ctx, &lsm6dsm_config.g_fs);
  ret *= lsm6dsm_gy_data_rate_get(dev_ctx, &lsm6dsm_config.g_odr);
  ret *= lsm6dsm_fifo_gy_batch_get(dev_ctx, &lsm6dsm_config.g_fifo_dec);  
  
  if (!ret)
    return IMU_STATUS_ERROR_COMM;
  
  diagnostic = 0x0;
  setup = true;
  
  return IMU_STATUS_OK;
}

imu_op_status_e lsm6dsm_processor (void)
{
  int i;
  axis3bit16_t data_raw_acceleration;
  axis3bit16_t data_raw_angular_rate;
  axis1bit16_t data_raw_temperature;
  
  lsm6dsm_reg_t reg;
  lsm6dsm_ctx_t *dev_ctx = &lsm6dsm_config.dev_ctx;
    
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
    ret = lsm6dsm_setup();
    if (ret != IMU_STATUS_OK) {
        return ret;
    }
  }
  
  /*
   * Read output only if new value is available.
   */
  if (!lsm6dsm_status_reg_get(dev_ctx, &reg.status_reg)) {
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
    if (!lsm6dsm_acceleration_raw_get(dev_ctx, data_raw_acceleration.u8bit)) {
      return IMU_STATUS_ERROR_COMM;
    }
    xl_fs_to_mg(data_raw_acceleration, lsm6dsm_data.acceleration_mg, lsm6dsm_config.xl_fs);

    // Adjust IMU orientation
    _acc_orientation(lsm6dsm_data.acceleration_mg);
  } 
  if (reg.status_reg.gda)
  {
    /*
     * Read angular rate field data.
     */
    for (i=0; i<3; i++) {
        data_raw_angular_rate.i16bit[i] = 0x0;
    }
    if (!lsm6dsm_angular_rate_raw_get(dev_ctx, data_raw_angular_rate.u8bit)) {
      return IMU_STATUS_ERROR_COMM;
    }
    gy_fs_to_mdps(data_raw_angular_rate, lsm6dsm_data.angular_rate_mdps, lsm6dsm_config.g_fs);
    
    // Adjust IMU orientation
    _gyr_orientation(lsm6dsm_data.angular_rate_mdps);
  }
  if (reg.status_reg.tda)
  {   
    /*
     * Read temperature data.
     */
    data_raw_temperature.i16bit = 0x0;
    if (!lsm6dsm_temperature_raw_get(dev_ctx, data_raw_temperature.u8bit)) {
      return IMU_STATUS_ERROR_COMM;
    }
    temp_fs_to_degc(data_raw_temperature, &(lsm6dsm_data.temperature_degC));
  }
  
  diagnostic = 0x0;

  if ((lsm6dsm_data.temperature_degC < MIN_TEMPERATURE_DEG) ||
      (lsm6dsm_data.temperature_degC > MAX_TEMPERATURE_DEG)) {
        diagnostic |= IMU_DIAGNOSTIC_TEMPP_ERROR;
        diagnostic |= IMU_DIAGNOSTIC_TEMPR_ERROR;
        diagnostic |= IMU_DIAGNOSTIC_TEMPY_ERROR;
  }

  return IMU_STATUS_OK;
}

imu_op_status_e lsm6dsm_get_diagnostic (uint32_t* oDiag)
{  
  *oDiag = diagnostic;
  
  return IMU_STATUS_OK;
}

imu_op_status_e lsm6dsm_get_imu_raw_values (imu_raw_data_struct_t* oRawValues)
{  
  oRawValues->Ax_mpss = lsm6dsm_data.acceleration_mg[0] / 1000.0f * ECUATORIAL_EFFECTIVE_GRAVITY;
  oRawValues->Ay_mpss = lsm6dsm_data.acceleration_mg[1] / 1000.0f * ECUATORIAL_EFFECTIVE_GRAVITY;
  oRawValues->Az_mpss = lsm6dsm_data.acceleration_mg[2] / 1000.0f * ECUATORIAL_EFFECTIVE_GRAVITY;
  oRawValues->Gp_rps = lsm6dsm_data.angular_rate_mdps[0] / 1000.0f * DEG_TO_RAD;
  oRawValues->Gr_rps = lsm6dsm_data.angular_rate_mdps[1] / 1000.0f * DEG_TO_RAD;
  oRawValues->Gy_rps = lsm6dsm_data.angular_rate_mdps[2] / 1000.0f * DEG_TO_RAD;
  
  return IMU_STATUS_OK;
}

imu_op_status_e lsm6dsm_get_imu_temperature (real32_T* oTemperature)
{
  *oTemperature = lsm6dsm_data.temperature_degC;
  
  return IMU_STATUS_OK;
}

imu_op_status_e lsm6dsm_get_imu_temperatures (imu_temperature_struct_t* oTemperatures)
{
  oTemperatures->Ax_Temp_degC = lsm6dsm_data.temperature_degC;
  oTemperatures->Ay_Temp_degC = lsm6dsm_data.temperature_degC;
  oTemperatures->Az_Temp_degC = lsm6dsm_data.temperature_degC;
  oTemperatures->Gp_Temp_degC = lsm6dsm_data.temperature_degC;
  oTemperatures->Gr_Temp_degC = lsm6dsm_data.temperature_degC;
  oTemperatures->Gy_Temp_degC = lsm6dsm_data.temperature_degC;
  
  return IMU_STATUS_OK;
}

imu_op_status_e lsm6dsm_get_config(lsm6dsm_config_struct_t **oConfig)
{
  *oConfig = &lsm6dsm_config;

  return IMU_STATUS_OK;
}
