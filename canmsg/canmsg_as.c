
/* Include ********************************************************************/
#include "canmsg\canmsg_as.h"
#include "include\imu_manager.h"

#include "j1939.h"
#include <canmsg\AS_canmsg.h>

#include <stdlib.h>

/* Defines ********************************************************************/
#define MODULE_SPECIFIC_ANALOG_STATUS_PGN   (PGN_ANALOG_STATUS | VS_MODULE_SPECIFIC_MSG)
/* Variables ******************************************************************/
static ANALOG_STATUS_T tAS;

/* Prototypes *****************************************************************/

/* Functions ******************************************************************/
void *CANMSG_GetPointer_AS(void) {
    return &tAS;
}

int32_t __Init_AS(uint32_t ctrl, uint32_t CycleTime)
{
  /* PGN 0xFF70 AS_canmsg - Analog Status */
  return Pgn_J1939 ( ctrl /*ctrl*/,
              MODULE_SPECIFIC_ANALOG_STATUS_PGN /*pgn*/,
              J1939_INFO_PRIORITY /*Priority*/,
              SIZE_ANALOG_STATUS /*DataLength*/,
              NULL /*pMsg*/,
              &tAS /*pMsg_Old*/,
              0U /*OnChangeTime*/,
              CycleTime /*CycleTime*/,
              __Update_AS/*(*Function)*/);
}

void __Update_AS(void)
{
  imu_temperature_struct_t data;
  imu_op_status_e status;
  
  status = IMU_MANAGER_GetIMUTemperatures(&data);
  
  tAS.Page = 100U;
  if (status != IMU_STATUS_OK) {
    tAS.Analog1Status = J1939SIG_NA_U16;
    tAS.Analog2Status = J1939SIG_NA_U16;
    tAS.Analog3Status = J1939SIG_NA_U16;
  } else {
    tAS.Analog1Status = (uint16_t) ((data.Gp_Temp_degC + 40) / 0.01f);
    tAS.Analog2Status = (uint16_t) ((data.Gr_Temp_degC + 40) / 0.01f);
    tAS.Analog3Status = (uint16_t) ((data.Gy_Temp_degC + 40) / 0.01f);
  }
	
  Output_J1939 (	0,
                  MODULE_SPECIFIC_ANALOG_STATUS_PGN,
                  J1939_INFO_PRIORITY,
                  SIZE_ANALOG_STATUS,
                  &tAS,
                  J1939_Address[0]);
  
  tAS.Page = 101U;
  if (status != IMU_STATUS_OK) {
    tAS.Analog1Status = J1939SIG_NA_U16;
    tAS.Analog2Status = J1939SIG_NA_U16;
    tAS.Analog3Status = J1939SIG_NA_U16;
  } else {
    tAS.Analog1Status = (uint16_t) ((data.Ax_Temp_degC + 40) / 0.01f);
    tAS.Analog2Status = (uint16_t) ((data.Ay_Temp_degC + 40) / 0.01f);
    tAS.Analog3Status = (uint16_t) ((data.Az_Temp_degC + 40) / 0.01f);
  }
}

/* End of $Workfile: canmsg_ari.c$ */
