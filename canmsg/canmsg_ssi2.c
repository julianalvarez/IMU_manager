
/* Include ********************************************************************/
#include "canmsg\canmsg_ssi2.h"
#include "include\imu_filter.h"

#include "j1939.h"
#include <canmsg\SSI2_canmsg.h>

#include <stdlib.h>

/* Defines ********************************************************************/

/* Variables ******************************************************************/
static SLOPE_SENSOR_INFORMATION_2_T tSSI2;
static bool reset_SSI2;

/* Prototypes *****************************************************************/

/* Functions ******************************************************************/
void *CANMSG_GetPointer_SSI2(void) {
    return &tSSI2;
}

int32_t __Init_SSI2(uint32_t ctrl, uint32_t CycleTime)
{
  /* PGN 0xF02A ARI_canmsg - Angular Rate Information (1 Hz) */
  return Pgn_J1939 ( ctrl /*ctrl*/,
              PGN_SLOPE_SENSOR_INFORMATION_2 /*pgn*/,
              J1939_INFO_PRIORITY /*Priority*/,
              SIZE_SLOPE_SENSOR_INFORMATION_2 /*DataLength*/,
              NULL /*pMsg*/,
              &tSSI2 /*pMsg_Old*/,
              0U /*OnChangeTime*/,
              CycleTime /*CycleTime*/,
              __Update_SSI2/*(*Function)*/);
}

void __Update_SSI2(void)
{
  imu_filter_data_struct_t data;
  imu_op_status_e status1, status2;
  bool filterActive;
  
  status1 = IMU_FILTER_GetValues(&data);
  status2 = IMU_FILTER_GetFilterActive(&filterActive);

  if ((status1 != IMU_STATUS_OK) || (status2 != IMU_STATUS_OK) || reset_SSI2) {
    tSSI2.PitchAngle = 0xFFFFFFU;
    tSSI2.RollAngle = 0xFFFFFFU;
    tSSI2.PitchAngleCompensation = SLOPE_SENSOR_INFORMATION_2_COMPENSATION_ERROR;
    tSSI2.PitchAngleFigureOfMerit = SLOPE_SENSOR_INFORMATION_2_FIGURE_OF_MERIT_ERROR;
    tSSI2.RollAngleCompensation = SLOPE_SENSOR_INFORMATION_2_COMPENSATION_ERROR;
    tSSI2.RollAngleFigureOfMerit = SLOPE_SENSOR_INFORMATION_2_FIGURE_OF_MERIT_ERROR;
    tSSI2.RollAndPitchMeasurementLatency = J1939SIG_NA_U8;
  } else {
    tSSI2.PitchAngle = (uint32_t) ((data.pitchAngle_rad * RAD_TO_DEG + 250.0f) * 32768.0f);
    tSSI2.RollAngle = (uint32_t) ((data.rollAngle_rad * RAD_TO_DEG + 250.0f) * 32768.0f);
    tSSI2.RollAndPitchMeasurementLatency = (uint8_t) 0U;

    if (!filterActive) {
      tSSI2.PitchAngleCompensation = SLOPE_SENSOR_INFORMATION_2_COMPENSATION_OFF;
      tSSI2.PitchAngleFigureOfMerit = SLOPE_SENSOR_INFORMATION_2_FIGURE_OF_MERIT_DEGRADED;
      tSSI2.RollAngleCompensation = SLOPE_SENSOR_INFORMATION_2_COMPENSATION_OFF;
      tSSI2.RollAngleFigureOfMerit = SLOPE_SENSOR_INFORMATION_2_FIGURE_OF_MERIT_DEGRADED;
    } else {
      tSSI2.PitchAngleCompensation = SLOPE_SENSOR_INFORMATION_2_COMPENSATION_ON;
      tSSI2.PitchAngleFigureOfMerit = SLOPE_SENSOR_INFORMATION_2_FIGURE_OF_MERIT_FULLY_FUNCTIONAL;
      tSSI2.RollAngleCompensation = SLOPE_SENSOR_INFORMATION_2_COMPENSATION_ON;
      tSSI2.RollAngleFigureOfMerit = SLOPE_SENSOR_INFORMATION_2_FIGURE_OF_MERIT_FULLY_FUNCTIONAL;
    }
  }
  
  reset_SSI2 = false;
  
}

void __Reset_SSI2(void)
{
    reset_SSI2 = true;
}

/* End of $Workfile: canmsg_ssi2.c$ */
