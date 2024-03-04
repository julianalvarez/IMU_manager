/*******************************************************************************
** @File: canmsg_as.h
** @Revision:
** @Date: 
** @Author: dlipicar
**                                 COPYRIGHT (C) FORKWORKS. All rights reserved.
*******************************************************************************/
#ifndef CANMSG_AS_H
#define CANMSG_AS_H
/* Include ********************************************************************/
#include "types.h"

/* Defines ********************************************************************/
#define ANALOG_STATUS_DEFAULT_CYCLE_TIME_MS              (0U)

/* Variables ******************************************************************/

/* Prototypes *****************************************************************/
void            *CANMSG_GetPointer_AS(void);
int32_t         __Init_AS(uint32_t ctrl, uint32_t CycleTime);
void            __Update_AS(void);

#endif /*CANMSG_AS_H*/
/* End of $Workfile: canmsg_as.h$ */
