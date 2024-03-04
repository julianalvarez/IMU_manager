
#ifndef CANMSG_PROPA_IMU_H
#define CANMSG_PROPA_IMU_H
/* Include ********************************************************************/
#include "types.h"
#include "canmsg/propa_canmsg.h"

/* Defines ********************************************************************/
/* Variables ******************************************************************/

/* Prototypes *****************************************************************/
int32_t         PROPA_IMU_Init(uint32_t ctrl, _Bool registerOnCommand, _Bool sendRequest);
uint32_t        PROPA_IMU_OnCommand(uint32_t Addr, PROPRIETARY_A_U* tPROPA);

#endif /*CANMSG_PROPA_IMU_H*/
/* End of $Workfile: canmsg_propa_imu.h$ */
