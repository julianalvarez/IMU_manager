
/* Includes ------------------------------------------------------------------*/
#include "drivers\imu_adc8344\imu_adc8344.h"

/* Externs ------------------------------------------------------------------*/
extern imu_op_status_e _ext_imu_adc8344_init (imu_hal_struct_t *iHal);
extern imu_op_status_e _ext_imu_adc8344_processor (void);

extern imu_op_status_e _ext_imu_adc8344_get_diagnostic (uint32_t* oDiag);
extern imu_op_status_e _ext_imu_adc8344_get_imu_raw_values (imu_raw_data_struct_t* oRawValues);
extern imu_op_status_e _ext_imu_adc8344_get_imu_temperature (real32_T* oTemperature);
extern imu_op_status_e _ext_imu_adc8344_get_imu_temperatures (imu_temperature_struct_t* oTemperatures);

/* Statics ********************************************************************/

/* Defines ********************************************************************/

/* Private functions ---------------------------------------------------------*/

/* Public functions ---------------------------------------------------------*/
imu_op_status_e imu_adc8344_init (imu_hal_struct_t *iHal)
{
  return _ext_imu_adc8344_init(iHal);
}

imu_op_status_e imu_adc8344_processor (void)
{
  return _ext_imu_adc8344_processor();
}

imu_op_status_e imu_adc8344_get_diagnostic (uint32_t* oDiag)
{  
  return _ext_imu_adc8344_get_diagnostic(oDiag);
}

imu_op_status_e imu_adc8344_get_imu_raw_values (imu_raw_data_struct_t* oRawValues)
{  
  return _ext_imu_adc8344_get_imu_raw_values(oRawValues);
}

imu_op_status_e imu_adc8344_get_imu_temperature (real32_T* oTemperature)
{
  return _ext_imu_adc8344_get_imu_temperature(oTemperature);
}

imu_op_status_e imu_adc8344_get_imu_temperatures (imu_temperature_struct_t* oTemperatures)
{
  return _ext_imu_adc8344_get_imu_temperatures(oTemperatures);
}
