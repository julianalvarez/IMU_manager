
/* Includes ------------------------------------------------------------------*/
#include "include\imu_driver.h"
#include "drivers\asm330lhh\asm330lhh_STdC\driver\asm330lhh_reg.h"

#ifndef _ASM330LHH_DRIVER_H
#define _ASM330LHH_DRIVER_H

typedef enum {
  ASM330LHH_STATUS_OK                        =  0,
  ASM330LHH_STATUS_ERROR_COMM                =  1,
  ASM330LHH_STATUS_ERROR_BAD_WHOAMI          =  2,
  ASM330LHH_STATUS_ERROR_BAD_CONFIG_MODE     =  3,
  ASM330LHH_STATUS_ERROR_NO_INIT             =  4,
  ASM330LHH_STATUS_ERROR_NO_SETUP            =  5
} asm330lhh_status;

typedef enum {
  ASM330LHH_CONFIG_MODE_DEFAULT       =  0,
  ASM330LHH_CONFIG_MODE_PILOT         =  1,
  ASM330LHH_CONFIG_MODE_ECUROW        =  2,
  ASM330LHH_CONFIG_MODE_SMARTANTENNA  =  3,
  ASM330LHH_CONFIG_MODE_LPC43xx       =  4,
  
} asm330lhh_config_mode;

typedef enum {
  ASM330LHH_HAL_STATUS_FAILED         =  0,
  ASM330LHH_HAL_STATUS_OK             =  1
} asm330lhh_hal_status;

/**
  * @brief  Device Configuration structure
  *
*/
typedef struct {
    asm330lhh_ctx_t               dev_ctx;
    
    asm330lhh_fifo_mode_t         fifo_mode;
    uint8_t                       bdu;
    
    asm330lhh_fs_xl_t             xl_fs;
    asm330lhh_odr_xl_t            xl_odr;
    asm330lhh_hp_slope_xl_en_t    xl_hp_slope_en;
    
    asm330lhh_fs_g_t              g_fs;
    asm330lhh_odr_g_t             g_odr;
} asm330lhh_config_struct_t;

/**
  * @brief  Data structure
  *
*/
typedef struct {
  float                     acceleration_mg[3];     //[x, y, z]
  float                     angular_rate_mdps[3];   //[pitch_rate, roll_rate, yaw_rate]
  float                     temperature_degC;
} asm330lhh_data_struct_t;

/* Public functions ---------------------------------------------------------*/
imu_op_status_e asm330lhh_init (imu_hal_struct_t *iHal);
imu_op_status_e asm330lhh_setup (void);
imu_op_status_e asm330lhh_processor (void);

imu_op_status_e asm330lhh_get_diagnostic (uint32_t* oDiag);
imu_op_status_e asm330lhh_get_imu_raw_values (imu_raw_data_struct_t* oRawValues);
imu_op_status_e asm330lhh_get_imu_temperature (real32_T* oTemperature);
imu_op_status_e asm330lhh_get_imu_temperatures (imu_temperature_struct_t* oTemperatures);

imu_op_status_e asm330lhh_get_config(asm330lhh_config_struct_t **oConfig);

#endif /*_ASM330LHH_DRIVER_H */
