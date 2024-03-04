
#ifndef CANMSG_ARI_H
#define CANMSG_ARI_H
/* Include ********************************************************************/
#include "types.h"

/* Defines ********************************************************************/
#define ANGULAR_RATE_INFORMATION_DEFAULT_CYCLE_TIME_MS              (1000U)

/* Variables ******************************************************************/

/* Prototypes *****************************************************************/
void            *CANMSG_GetPointer_ARI(void);
int32_t         __Init_ARI(uint32_t ctrl, uint32_t CycleTime);
void            __Update_ARI(void);
void            __Reset_ARI(void);

#endif /*CANMSG_ARI_H*/
/* End of $Workfile: canmsg_ari.h$ */
