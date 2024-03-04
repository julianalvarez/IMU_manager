
/* Include ********************************************************************/
#include "canmsg\canmsg_propa.h"
#include "imu_manager.h"

#include "j1939.h"
#include <canmsg/PROPA_canmsg.h>
#include <canmsg/PROPA_IMU_canmsg.h>
#include <string.h>

/* Defines ********************************************************************/
void _processGetParameter(uint32_t Addr, uint16_t paramIndex);
void _processSetParameter(uint32_t Addr, uint16_t paramIndex, real32_T paramValue);
void _processRestoreDefaults(uint32_t Addr, uint8_t checkValue, uint8_t command);
void _processCommitData(uint32_t Addr, uint8_t checkValue);

void _sendPROPA(PROPRIETARY_A_U* propa, uint32_t Addr);

/* Variables ******************************************************************/
static PROPRIETARY_A_U tPROPA_IMU;

/* Prototypes *****************************************************************/
uint32_t  __OnCommand_PROPA_IMU(uint32_t Addr);

/* Functions ******************************************************************/
void _dummy(uint32_t a, uint32_t b)
{
    a = a;
    b = b;
}

int32_t PROPA_IMU_Init(uint32_t ctrl, _Bool registerOnCommand, _Bool sendRequest)
{
    int32_t result;

    if (registerOnCommand) {
    result = OnPgn_J1939(   ctrl, 
                            PGN_PROPRIETARY_A, 
                            SIZE_PROPRIETARY_A, 
                            &tPROPA_IMU, 
                            __OnCommand_PROPA_IMU, 
                            0, 
                            NULL);
        if (result != RC_SUCCESS) {
            return result;
        }
    }
  
    if (sendRequest) {
        // PropA request every time the module starts to ask for configuration
        result = Request_J1939 (ctrl,
                                PGN_PROPRIETARY_A,
                                J1939_ADDRESS_GLOBAL, 
                                _dummy,  
                                0U,
                                NULL);
    }
  
    return result;
}

uint32_t PROPA_IMU_OnCommand(uint32_t Addr, PROPRIETARY_A_U* tPROPA)
{
    memcpy ((void *)&tPROPA_IMU, (void *)tPROPA, SIZE_PROPRIETARY_A); // Copy message
    
    return __OnCommand_PROPA_IMU(Addr);
}

uint32_t  __OnCommand_PROPA_IMU(uint32_t Addr)
{  
  if (tPROPA_IMU.P140.IMU_MSG_P140.Page == IMU_MSG_PAGE) {
    switch (tPROPA_IMU.P140.IMU_MSG_P140.XID) {
      case IMU_MSG_XID_GET_PARAMETER:
        _processGetParameter(Addr, tPROPA_IMU.P140.IMU_MSG_P140_GET_PARAMETER.ParameterIndex);
        break;
      case IMU_MSG_XID_SET_PARAMETER:
        _processSetParameter(Addr, tPROPA_IMU.P140.IMU_MSG_P140_SET_PARAMETER.ParameterIndex, tPROPA_IMU.P140.IMU_MSG_P140_SET_PARAMETER.ParameterValue);
        break;
      case IMU_MSG_XID_RESTORE_DEFAULTS:
        _processRestoreDefaults(Addr, tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS.CheckValue, tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS.Command);
        break;
      case IMU_MSG_XID_COMMIT_DATA:
        _processCommitData(Addr, tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA.CheckValue);
        break;
    }
  }
  return 0;
}

void _processGetParameter(uint32_t Addr, uint16_t paramIndex)
{
  uint8_t bParam;
  real32_T rParam;
  
  bool ok = false;
  real32_T tmp;
  
  switch (paramIndex) {
    case IMU_PARAMETER_MOUNTING_POSITION:
      if (IMU_MANAGER_GetMountingPosition(&bParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = bParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_QW:
      if (IMU_MANAGER_GetMountingQuaternionQw(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_QX:
      if (IMU_MANAGER_GetMountingQuaternionQx(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_QY:
      if (IMU_MANAGER_GetMountingQuaternionQy(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_QZ:
      if (IMU_MANAGER_GetMountingQuaternionQz(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_TEMPERATURE:
      if (IMU_MANAGER_GetIMUTemperature(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_TEMPERATURE_AX:
      if (IMU_MANAGER_GetIMUTempAx(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_TEMPERATURE_AY:
      if (IMU_MANAGER_GetIMUTempAy(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_TEMPERATURE_AZ:
      if (IMU_MANAGER_GetIMUTempAz(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_TEMPERATURE_GP:
      if (IMU_MANAGER_GetIMUTempGp(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_TEMPERATURE_GR:
      if (IMU_MANAGER_GetIMUTempGr(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_TEMPERATURE_GY:
      if (IMU_MANAGER_GetIMUTempGy(&rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
    default:
      if (IMU_MANAGER_GetCalibrationParameter((imu_calibration_parameters_e) paramIndex, &rParam) == IMU_STATUS_OK) {
        ok = true;
        tmp = rParam;
      }
      break;
  }
  
  if (ok) {
    tPROPA_IMU.P140.IMU_MSG_P140_GET_PARAMETER_RESPONSE.Page = IMU_MSG_PAGE;
    tPROPA_IMU.P140.IMU_MSG_P140_GET_PARAMETER_RESPONSE.XID = IMU_MSG_XID_GET_PARAMETER_RESPONSE;
    tPROPA_IMU.P140.IMU_MSG_P140_GET_PARAMETER_RESPONSE.ParameterIndex = paramIndex;
    tPROPA_IMU.P140.IMU_MSG_P140_GET_PARAMETER_RESPONSE.ParameterValue = tmp;
    
    _sendPROPA(&tPROPA_IMU, Addr);
  }
}

void _processSetParameter(uint32_t Addr, uint16_t paramIndex, real32_T paramValue)
{
  uint8_t bParam;
  real32_T rParam;
  
  bool ok = false;
  real32_T tmp;
  
  switch (paramIndex) {
    case IMU_PARAMETER_MOUNTING_POSITION:
      if ((IMU_MANAGER_SetMountingPosition(paramValue) == IMU_STATUS_OK) && (IMU_MANAGER_GetMountingPosition(&bParam) == IMU_STATUS_OK)) {
        ok = true;
        tmp = bParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_QW:
      if ((IMU_MANAGER_SetMountingQuaternionQw(paramValue) == IMU_STATUS_OK) && (IMU_MANAGER_GetMountingQuaternionQw(&rParam) == IMU_STATUS_OK)) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_QX:
      if ((IMU_MANAGER_SetMountingQuaternionQx(paramValue) == IMU_STATUS_OK) && (IMU_MANAGER_GetMountingQuaternionQx(&rParam) == IMU_STATUS_OK)) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_QY:
      if ((IMU_MANAGER_SetMountingQuaternionQy(paramValue) == IMU_STATUS_OK) && (IMU_MANAGER_GetMountingQuaternionQy(&rParam) == IMU_STATUS_OK)) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_QZ:
      if ((IMU_MANAGER_SetMountingQuaternionQz(paramValue) == IMU_STATUS_OK) && (IMU_MANAGER_GetMountingQuaternionQz(&rParam) == IMU_STATUS_OK)) {
        ok = true;
        tmp = rParam;
      }
      break;
    case IMU_PARAMETER_MOUNTING_QUATERNION_END:
      if ((IMU_MANAGER_SetMountingQuaternionEnd() == IMU_STATUS_OK)) {
        ok = true;
        tmp = 0;
      }
      break;
    default:
      if ((IMU_MANAGER_SetCalibrationParameter((imu_calibration_parameters_e) paramIndex, paramValue) == IMU_STATUS_OK) && (IMU_MANAGER_GetCalibrationParameter((imu_calibration_parameters_e) paramIndex, &rParam) == IMU_STATUS_OK)) {
        ok = true;
        tmp = rParam;
      }
      break;
  }
  
  if (ok) {
    tPROPA_IMU.P140.IMU_MSG_P140_SET_PARAMETER_RESPONSE.Page = IMU_MSG_PAGE;
    tPROPA_IMU.P140.IMU_MSG_P140_SET_PARAMETER_RESPONSE.XID = IMU_MSG_XID_SET_PARAMETER_RESPONSE;
    tPROPA_IMU.P140.IMU_MSG_P140_SET_PARAMETER_RESPONSE.ParameterIndex = paramIndex;
    tPROPA_IMU.P140.IMU_MSG_P140_SET_PARAMETER_RESPONSE.ParameterValue = tmp;
    
    _sendPROPA(&tPROPA_IMU, Addr);
  }
}

void _processRestoreDefaults(uint32_t Addr, uint8_t checkValue, uint8_t command)
{
  imu_op_status_e ret = IMU_STATUS_ERROR_BAD_PARAM_ID;
  if (checkValue == IMU_MSG_RESTORE_DEFAULTS_CHECK_VALUE) {
    if (command == IMU_MSG_RESTORE_DEFAULTS_COMMAND_ACC) {
      ret = IMU_MANAGER_RestoreDefaultConfigurationAcc();
    } else if (command == IMU_MSG_RESTORE_DEFAULTS_COMMAND_GYR) {
      ret = IMU_MANAGER_RestoreDefaultConfigurationGyr();
    } else if (command == IMU_MSG_RESTORE_DEFAULTS_COMMAND_FULL) {
      ret = IMU_MANAGER_RestoreDefaultConfigurationFull();
    }
    
    if (ret == IMU_STATUS_OK) {
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.Page = IMU_MSG_PAGE;
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.XID = IMU_MSG_XID_RESTORE_DEFAULTS_RESPONSE;
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.RestoreStatus = IMU_MSG_RESTORE_DEFAULTS_RESPONSE_STATUS_SUCCESS;
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.Rvd1 = J1939SIG_NA_U8;
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.Rvd2 = J1939SIG_NA_U32;
    } else {
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.Page = IMU_MSG_PAGE;
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.XID = IMU_MSG_XID_RESTORE_DEFAULTS_RESPONSE;
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.RestoreStatus = IMU_MSG_RESTORE_DEFAULTS_RESPONSE_STATUS_FAIL;
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.Rvd1 = J1939SIG_NA_U8;
      tPROPA_IMU.P140.IMU_MSG_P140_RESTORE_DEFAULTS_RESPONSE.Rvd2 = J1939SIG_NA_U32;
    }
    
    _sendPROPA(&tPROPA_IMU, Addr);
  }
}

void _processCommitData(uint32_t Addr, uint8_t checkValue)
{
  if (checkValue == IMU_MSG_COMMIT_DATA_CHECK_VALUE) {
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Page = IMU_MSG_PAGE;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.XID = IMU_MSG_XID_COMMIT_DATA_RESPONSE;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.CommitStatus = IMU_MSG_COMMIT_DATA_RESPONSE_STATUS_READY_TO_COMMIT;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.CommitErrorCode = J1939SIG_NA_U16;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Rvd1 = J1939SIG_NA_U8;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Rvd2 = J1939SIG_NA_U16;
    _sendPROPA(&tPROPA_IMU, Addr);
    
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Page = IMU_MSG_PAGE;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.XID = IMU_MSG_XID_COMMIT_DATA_RESPONSE;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.CommitStatus = IMU_MSG_COMMIT_DATA_RESPONSE_STATUS_COMMIT_BEGUN;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.CommitErrorCode = J1939SIG_NA_U16;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Rvd1 = J1939SIG_NA_U8;
    tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Rvd2 = J1939SIG_NA_U16;
    _sendPROPA(&tPROPA_IMU, Addr);
    
    if (IMU_MANAGER_PersistConfiguration() == IMU_STATUS_OK) {
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Page = IMU_MSG_PAGE;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.XID = IMU_MSG_XID_COMMIT_DATA_RESPONSE;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.CommitStatus = IMU_MSG_COMMIT_DATA_RESPONSE_STATUS_COMMIT_SUCCESS;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.CommitErrorCode = J1939SIG_NA_U16;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Rvd1 = J1939SIG_NA_U8;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Rvd2 = J1939SIG_NA_U16;
    } else {
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Page = IMU_MSG_PAGE;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.XID = IMU_MSG_XID_COMMIT_DATA_RESPONSE;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.CommitStatus = IMU_MSG_COMMIT_DATA_RESPONSE_STATUS_COMMIT_FAIL;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.CommitErrorCode = J1939SIG_NA_U16;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Rvd1 = J1939SIG_NA_U8;
      tPROPA_IMU.P140.IMU_MSG_P140_COMMIT_DATA_RESPONSE.Rvd2 = J1939SIG_NA_U16;
    }
    
    _sendPROPA(&tPROPA_IMU, Addr);
  }
}

void _sendPROPA(PROPRIETARY_A_U* propa, uint32_t Addr)
{
    Output_J1939(0,
                 PGN_PROPRIETARY_A | (0xFF & Addr),
                 J1939_INFO_PRIORITY,
                 SIZE_PROPRIETARY_A,
                 propa,
                 J1939_Address[0]);
}

/* End of $Workfile: canmsg_propa.c$ */
