
/* Includes ------------------------------------------------------------------*/
#include "include\imu_driver.h"

#ifndef _IMU_ADC8344_DRIVER_H
#define _IMU_ADC8344_DRIVER_H

/* Public functions ---------------------------------------------------------*/
imu_op_status_e imu_adc8344_init (imu_hal_struct_t *iHal);
imu_op_status_e imu_adc8344_processor (void);

imu_op_status_e imu_adc8344_get_diagnostic (uint32_t* oDiag);
imu_op_status_e imu_adc8344_get_imu_raw_values (imu_raw_data_struct_t* oRawValues);
imu_op_status_e imu_adc8344_get_imu_temperature (real32_T* oTemperature);
imu_op_status_e imu_adc8344_get_imu_temperatures (imu_temperature_struct_t* oTemperatures);

#endif /*_IMU_ADC8344_DRIVER_H */
