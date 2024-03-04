
/* Include ********************************************************************/
#include "types.h"

#include <j1939.h>

#include <include\imu_canmsg.h>
#include "canmsg\canmsg_propa.h"
#include "canmsg\canmsg_accs.h"
#include "canmsg\canmsg_ari.h"
#include "canmsg\canmsg_ssi2.h"
#include "canmsg\canmsg_as.h"

/* Defines ********************************************************************/
/* Globals ********************************************************************/
/* Statics ********************************************************************/
/* Prototypes *****************************************************************/
/* Functions ******************************************************************/
void IMU_CANMSG_Init(uint32_t ctrl, _Bool registerOnCommand, _Bool sendRequest, uint32_t ARI_CycleTime, uint32_t ACCS_CycleTime, uint32_t SSI2_CycleTime, uint32_t AS_CycleTime)
{
    uint32_t    result;
    
    result = __Init_ARI(ctrl, ARI_CycleTime);
    if (result != RC_SUCCESS) {
#ifdef REPORT_ERRORS_THROUGH_UART
		printf ("Error Registering Message ARI\n");
#endif
        while(1){}
	}
    
    result = __Init_ACCS(ctrl, ACCS_CycleTime);
    if (result != RC_SUCCESS) {
#ifdef REPORT_ERRORS_THROUGH_UART
		printf ("Error Registering Message ACCS\n");
#endif
        while(1){}
	}
    
    result = __Init_SSI2(ctrl, SSI2_CycleTime);
    if (result != RC_SUCCESS) {
#ifdef REPORT_ERRORS_THROUGH_UART
		printf ("Error Registering Message ACCS\n");
#endif
        while(1){}
	}

    result = __Init_AS(ctrl, AS_CycleTime);
    if (result != RC_SUCCESS) {
#ifdef REPORT_ERRORS_THROUGH_UART
		printf ("Error Registering Message AS\n");
#endif
        while(1){}
	}
    
  result = PROPA_IMU_Init(ctrl, registerOnCommand, sendRequest);
  if (result != RC_SUCCESS) {
#ifdef REPORT_ERRORS_THROUGH_UART
		printf ("Error Registering Message PROPA\n");
#endif
        while(1){}
  }
}

uint32_t IMU_CANMSG_PROPA_OnCommand(uint32_t Addr, PROPRIETARY_A_U* tPROPA)
{
    return PROPA_IMU_OnCommand(Addr, tPROPA);
}

void IMU_CANMSG_Reset(void)
{
    __Reset_ARI();
    __Reset_ACCS();
    __Reset_SSI2();
}

