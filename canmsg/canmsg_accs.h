
#ifndef CANMSG_ACCS_H
#define CANMSG_ACCS_H
/* Include ********************************************************************/
#include "types.h"

/* Defines ********************************************************************/
#define ACCELERATION_SENSOR_DEFAULT_CYCLE_TIME_MS              (1000U)

/* Variables ******************************************************************/

/* Prototypes *****************************************************************/
void            *CANMSG_GetPointer_ACCS(void);
int32_t         __Init_ACCS(uint32_t ctrl, uint32_t CycleTime);
void            __Update_ACCS(void);
void            __Reset_ACCS(void);

#endif /*CANMSG_ACCS_H*/
/* End of $Workfile: canmsg_accs.h$ */
