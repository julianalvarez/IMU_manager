
#ifndef _IMU_HAL_H
#define _IMU_HAL_H

/* Includes ------------------------------------------------------------------*/
#include "types.h"

/* Defines ********************************************************************/

/* Enums **********************************************************************/
typedef enum {
  IMU_HAL_STATUS_FAILED         =  0,
  IMU_HAL_STATUS_OK             =  1
} imu_hal_status_e;

/* Typedefs *******************************************************************/

/**
  * @brief  Generic transfer operation.
  *         Performs a read+write or write+read operation on the IMU, regardless of the protocol used.
  *         This function is used to perform I/O operations on the device's registers
  *
  * @param  uint8_t *pCmdBuffer: pointer to data to send to IMU before the read operation
  * @param  uint16_t nCmdBufferSize: number of consecutive bytes to send
  * @param  uint8_t *pReadBuffer: pointer to data read from IMU
  * @param  uint16_t nReadBufferSize: number of consecutive bytes to read
  * @return lsm6dsm_hal_status: IMU_HAL_STATUS_FAILED if operation failed, IMU_HAL_STATUS_OK otherwise
*/
typedef imu_hal_status_e (*imu_hal_read_ptr) (uint8_t *pCmdBuffer, uint16_t nCmdBufferSize, uint8_t *pReadBuffer, uint16_t nReadBufferSize);

/**
  * @brief  Generic transfer operation.
  *         Performs a read+write or write+read operation on the IMU, regardless of the protocol used.
  *         This function is used to perform I/O operations on the device's registers
  *
  * @param  uint8_t *pCmdBuffer: pointer to data to send to IMU before the write operation
  * @param  uint16_t nCmdBufferSize: number of consecutive bytes to send
  * @param  uint8_t *pWriteBuffer: pointer to data to write to IMU
  * @param  uint16_t nWriteBufferSize: number of consecutive bytes to write
  * @return lsm6dsm_hal_status: IMU_HAL_STATUS_FAILED if operation failed, IMU_HAL_STATUS_OK otherwise
*/
typedef imu_hal_status_e (*imu_hal_write_ptr) (uint8_t *pCmdBuffer, uint16_t nCmdBufferSize, uint8_t *pWriteBuffer, uint16_t nWriteBufferSize);

/**
  * @brief  EEPROM read operation.
  *
  * @param  uint32_t nAddressOffset: Pointer offset to the address in which the read operation begins. Bounded by [0, EEPROM_IMU_SIZE-1]
  * @param  uint8_t *pReadBuffer: pointer to data read from EEPROM
  * @param  uint32_t nBufferSize: number of consecutive bytes to read
  * @return lsm6dsm_hal_status: IMU_HAL_STATUS_FAILED if operation failed, IMU_HAL_STATUS_OK otherwise
*/
typedef imu_hal_status_e (*imu_hal_eeprom_read_ptr) (uint32_t nAddressOffset, uint8_t *pReadBuffer, uint32_t nBufferSize);

/**
  * @brief  EEPROM write operation.
  *
  * @param  uint32_t nAddressOffset: Pointer offset to the address in which the write operation begins. Bounded by [0, EEPROM_IMU_SIZE-1]
  * @param  uint8_t *pWriteBuffer: pointer to data write to EEPROM
  * @param  uint32_t nBufferSize: number of consecutive bytes to write
  * @return lsm6dsm_hal_status: IMU_HAL_STATUS_FAILED if operation failed, IMU_HAL_STATUS_OK otherwise
*/
typedef imu_hal_status_e (*imu_hal_eeprom_write_ptr) (uint32_t nAddressOffset, uint8_t *pWriteBuffer, uint32_t nBufferSize);

/**
  * @brief  Sleep function.
  *
  * @param  uint32_t nDelayMs: Time to sleep in ms.
*/
typedef void (*imu_hal_sleep_ms_ptr) (uint32_t nDelayMs);

/**
  * @brief  Time get function.
  *
  * @return uint32_t: Time ellapsed since startup in ms.
  * @param  uint32_t nDelayMs: Time to sleep in ms.
*/
typedef uint32_t (*imu_hal_time_get_ptr) (void);

/**
  * @brief  HAL Configuration structure
  *
*/
typedef struct tag_imu_hal_struct {
  imu_hal_read_ptr                  pIMURead;
  imu_hal_write_ptr                 pIMUWrite;
  imu_hal_eeprom_read_ptr           pEEPROMRead;
  imu_hal_eeprom_write_ptr          pEEPROMWrite;
  imu_hal_sleep_ms_ptr              pSleepMs;
} imu_hal_struct_t;

/**
  * @brief  HAL Configuration structure
  *
*/
typedef struct tag_imu_filter_hal_struct {
  imu_hal_time_get_ptr              pTimeGet;
} imu_filter_hal_struct_t;
/* Externs ********************************************************************/

/* Prototypes *****************************************************************/

#endif /*_IMU_HAL_H */
