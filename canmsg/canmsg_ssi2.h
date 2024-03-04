
#ifndef CANMSG_SSI2_H
#define CANMSG_SSI2_H
/* Include ********************************************************************/
#include "types.h"

/* Defines ********************************************************************/
#define SLOPE_SENSOR_INFORMATION_2_DEFAULT_CYCLE_TIME_MS              (1000U)

/* Variables ******************************************************************/

/* Prototypes *****************************************************************/
void            *CANMSG_GetPointer_SSI2(void);
int32_t         __Init_SSI2(uint32_t ctrl, uint32_t CycleTime);
void            __Update_SSI2(void);
void            __Reset_SSI2(void);

#endif /*CANMSG_SSI2_H*/
/* End of $Workfile: canmsg_ssi2.h$ */
