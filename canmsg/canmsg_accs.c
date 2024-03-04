

/* Include ********************************************************************/
#include "canmsg\canmsg_accs.h"
#include "include\imu_manager.h"

#include "j1939.h"
#include <canmsg\ACCS_canmsg.h>

#include <stdlib.h>

/* Defines ********************************************************************/

/* Variables ******************************************************************/
static ACCELERATION_SENSOR_T tACCS;
static bool reset_ACCS;

/* Prototypes *****************************************************************/

/* Functions ******************************************************************/
void *CANMSG_GetPointer_ACCS(void) {
    return &tACCS;
}

int32_t __Init_ACCS(uint32_t ctrl, uint32_t CycleTime)
{
  /* PGN 0xF02D ACCS_canmsg - Acceleration Sensor (1 Hz) */
  return Pgn_J1939 ( ctrl /*ctrl*/,
              PGN_ACCELERATION_SENSOR /*pgn*/,
              J1939_INFO_PRIORITY /*Priority*/,
              SIZE_ACCELERATION_SENSOR /*DataLength*/,
              NULL /*pMsg*/,
              &tACCS /*pMsg_Old*/,
              0U /*OnChangeTime*/,
              CycleTime /*CycleTime*/,
              __Update_ACCS/*(*Function)*/);
}

void __Update_ACCS(void)
{
  imu_data_struct_t data;
  imu_op_status_e status;
  
  status = IMU_MANAGER_GetIMUValues(&data);

  if ((status != IMU_STATUS_OK) || reset_ACCS) {
    tACCS.LateralAccelerationExtendedRange = J1939SIG_NA_U16;
    tACCS.LongitudinalAccelerationExtendedRange = J1939SIG_NA_U16;
    tACCS.VerticalAccelerationExtendedRange = J1939SIG_NA_U16;
    tACCS.LateralAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_ERROR;
    tACCS.LongitudinalAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_ERROR;
    tACCS.VerticalAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_ERROR;
  } else {    
    tACCS.LateralAccelerationExtendedRange = (uint16_t) ((data.latAcc_mpss + 320.0f) * 100.0f);
    tACCS.LongitudinalAccelerationExtendedRange = (uint16_t) ((data.longAcc_mpss + 320.0f) * 100.0f);
    tACCS.VerticalAccelerationExtendedRange = (uint16_t) ((data.vertAcc_mpss + 320.0f) * 100.0f);
    
    if (!IMU_MANAGER_IsAccCalibrationValid()) {
      tACCS.LateralAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_DEGRADED;
      tACCS.LongitudinalAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_DEGRADED;
      tACCS.VerticalAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_DEGRADED;
    } else {
      tACCS.LateralAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_FULLY_FUNCTIONAL;
      tACCS.LongitudinalAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_FULLY_FUNCTIONAL;
      tACCS.VerticalAccelerationExtendedRangeFigureOfMerit = ACCELERATION_SENSOR_FIGURE_OF_MERIT_FULLY_FUNCTIONAL;
    }
  }
  
  tACCS.SupportVariableTransmissionRepetitionRate = J1939SIG_NA_U2;
  tACCS.Rvd1 = J1939SIG_NA_U8;
  
  reset_ACCS = false;
}

void __Reset_ACCS(void)
{
    reset_ACCS = true;
}

/* End of $Workfile: canmsg_accs.c$ */
