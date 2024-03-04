
/* Include ********************************************************************/
#include "canmsg\canmsg_ari.h"
#include "include\imu_manager.h"

#include "j1939.h"
#include <canmsg\ARI_canmsg.h>

#include <stdlib.h>

/* Defines ********************************************************************/

/* Variables ******************************************************************/
static ANGULAR_RATE_INFORMATION_T tARI;
static bool _reset_ARI;

/* Prototypes *****************************************************************/

/* Functions ******************************************************************/
void *CANMSG_GetPointer_ARI(void) {
    return &tARI;
}

int32_t __Init_ARI(uint32_t ctrl, uint32_t CycleTime)
{
  /* PGN 0xF02A ARI_canmsg - Angular Rate Information (1 Hz) */
  return Pgn_J1939 ( ctrl /*ctrl*/,
              PGN_ANGULAR_RATE_INFORMATION /*pgn*/,
              J1939_INFO_PRIORITY /*Priority*/,
              SIZE_ANGULAR_RATE_INFORMATION /*DataLength*/,
              NULL /*pMsg*/,
              &tARI /*pMsg_Old*/,
              0U /*OnChangeTime*/,
              CycleTime /*CycleTime*/,
              __Update_ARI/*(*Function)*/);
}

void __Update_ARI(void)
{
  imu_data_struct_t data;
  imu_op_status_e status;
  
  status = IMU_MANAGER_GetIMUValues(&data);

  if ((status != IMU_STATUS_OK) || _reset_ARI) {
    tARI.PitchRateExtendedRange = J1939SIG_NA_U16;
    tARI.RollRateExtendedRange = J1939SIG_NA_U16;
    tARI.YawRateExtendedRange = J1939SIG_NA_U16;
    tARI.PitchRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_ERROR;
    tARI.RollRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_ERROR;
    tARI.YawRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_ERROR;
  } else {
    tARI.PitchRateExtendedRange = (uint16_t) ((data.pitchRate_rps * RAD_TO_DEG + 250.0f) * 128.0f);
    tARI.RollRateExtendedRange = (uint16_t) ((data.rollRate_rps * RAD_TO_DEG + 250.0f) * 128.0f);
    tARI.YawRateExtendedRange = (uint16_t) ((data.yawRate_rps * RAD_TO_DEG + 250.0f) * 128.0f);
    
    if (!IMU_MANAGER_IsGyrCalibrationValid()) {
      tARI.PitchRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_DEGRADED;
      tARI.RollRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_DEGRADED;
      tARI.YawRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_DEGRADED;
    } else {
      tARI.PitchRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_FULLY_FUNCTIONAL;
      tARI.RollRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_FULLY_FUNCTIONAL;
      tARI.YawRateExtendedRangeFigureOfMerit = ANGULAR_RATE_INFORMATION_FIGURE_OF_MERIT_FULLY_FUNCTIONAL;
    }
  }
  
  tARI.Rvd1 = J1939SIG_NA_U2;
  tARI.Rvd2 = J1939SIG_NA_U8;
  
  _reset_ARI = false;
}

void __Reset_ARI(void)
{
    _reset_ARI = true;
}

/* End of $Workfile: canmsg_ari.c$ */
