
/* Includes ------------------------------------------------------------------*/
#include "include\imu_driver.h"
#include "drivers\lsm6dsm\lsm6dsm_STdC\driver\lsm6dsm_reg.h"

#ifndef _LSM6DSM_DRIVER_H
#define _LSM6DSM_DRIVER_H

typedef enum {
  LSM6DSM_STATUS_OK                        =  0,
  LSM6DSM_STATUS_ERROR_COMM                =  1,
  LSM6DSM_STATUS_ERROR_BAD_WHOAMI          =  2,
  LSM6DSM_STATUS_ERROR_BAD_CONFIG_MODE     =  3,
  LSM6DSM_STATUS_ERROR_NO_INIT             =  4,
  LSM6DSM_STATUS_ERROR_NO_SETUP            =  5
} lsm6dsm_status;

typedef enum {
  LSM6DSM_CONFIG_MODE_DEFAULT       =  0,
  LSM6DSM_CONFIG_MODE_PILOT         =  1,
  LSM6DSM_CONFIG_MODE_ECUROW        =  2,
  LSM6DSM_CONFIG_MODE_SMARTANTENNA  =  3,
  LSM6DSM_CONFIG_MODE_LPC43xx       =  4,
  
} lsm6dsm_config_mode;

typedef enum {
  LSM6DSM_HAL_STATUS_FAILED         =  0,
  LSM6DSM_HAL_STATUS_OK             =  1
} lsm6dsm_hal_status;

/**
  * @brief  Device Configuration structure
  *
*/
typedef struct {
    lsm6dsm_ctx_t               dev_ctx;
    
    lsm6dsm_fifo_mode_t         fifo_mode;
    uint8_t                     bdu;
    
    lsm6dsm_xl_hm_mode_t        xl_hm_mode;
    lsm6dsm_fs_xl_t             xl_fs;
    lsm6dsm_odr_xl_t            xl_odr;
    lsm6dsm_dec_fifo_xl_t       xl_fifo_dec;
    lsm6dsm_input_composite_t   xl_input_composite;
    
    lsm6dsm_g_hm_mode_t         g_hm_mode;
    lsm6dsm_fs_g_t              g_fs;
    lsm6dsm_odr_g_t             g_odr;
    lsm6dsm_dec_fifo_gyro_t     g_fifo_dec;
} lsm6dsm_config_struct_t;

/**
  * @brief  Data structure
  *
*/
typedef struct {
  float                     acceleration_mg[3];     //[x, y, z]
  float                     angular_rate_mdps[3];   //[pitch_rate, roll_rate, yaw_rate]
  float                     temperature_degC;
} lsm6dsm_data_struct_t;

/* Public functions ---------------------------------------------------------*/
imu_op_status_e lsm6dsm_init (imu_hal_struct_t *iHal);
imu_op_status_e lsm6dsm_setup (void);
imu_op_status_e lsm6dsm_processor (void);

imu_op_status_e lsm6dsm_get_diagnostic (uint32_t* oDiag);
imu_op_status_e lsm6dsm_get_imu_raw_values (imu_raw_data_struct_t* oRawValues);
imu_op_status_e lsm6dsm_get_imu_temperature (real32_T* oTemperature);
imu_op_status_e lsm6dsm_get_imu_temperatures (imu_temperature_struct_t* oTemperatures);

imu_op_status_e lsm6dsm_get_config(lsm6dsm_config_struct_t **oConfig);

#endif /*_LSM6DSM_DRIVER_H */
