
#ifndef _IMU_CANMSG_H_
#define _IMU_CANMSG_H_

#include <types.h>
#include <canmsg/propa_canmsg.h>

/* Defines ********************************************************************/

/* Enums **********************************************************************/

/* Externs ********************************************************************/

/* Prototypes *****************************************************************/
#if defined(__cplusplus)
extern "C"
{
#endif

void            IMU_CANMSG_Init(uint32_t ctrl, _Bool registerOnCommand, _Bool sendRequest, uint32_t ARI_CycleTime, uint32_t ACCS_CycleTime, uint32_t SSI2_CycleTime, uint32_t AS_CycleTime);
uint32_t        IMU_CANMSG_PROPA_OnCommand(uint32_t Addr, PROPRIETARY_A_U* tPROPA);
void            IMU_CANMSG_Reset(void);
    
#if defined(__cplusplus)
}
#endif

#endif // _IMU_CANMSG_H_
