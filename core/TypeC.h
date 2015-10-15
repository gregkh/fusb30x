/*********************************************************************
 * FileName:        TypeC.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC32
 * Compiler:        XC32
 * Company:         Fairchild Semiconductor
 *
 * Author           Date          Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * M. Smith         12/04/2014    Initial Version
 *
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Software License Agreement:
 *
 * The software supplied herewith by Fairchild Semiconductor (the “Company”)
 * is supplied to you, the Company's customer, for exclusive use with its
 * USB Type C / USB PD products.  The software is owned by the Company and/or
 * its supplier, and is protected under applicable copyright laws.
 * All rights are reserved. Any use in violation of the foregoing restrictions
 * may subject the user to criminal sanctions under applicable laws, as well
 * as to civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ********************************************************************/

#ifndef __FSC_TYPEC_H__
#define	__FSC_TYPEC_H__

/////////////////////////////////////////////////////////////////////////////
//                              Required headers
/////////////////////////////////////////////////////////////////////////////
#include "platform.h"
#include "fusb30X.h"
#include "PDPolicy.h"
#include "PDProtocol.h"
#include "TypeC_Types.h"

// Type C Timing Parameters
// Units are in ms * 10 to be ticked by a 0.1ms timer.
#define tAMETimeout     1010 * 10
#define tCCDebounceMin  100 * 10
#define tCCDebounceNom  120 * 10
#define tCCDebounceMax  200 * 10
#define tPDDebounceMin  10 * 10
#define tPDDebounceMax  20 * 10
#define tAccDetect      100 * 10
#define tDRP            80 * 10
#define tDRPAdvert      30 * 10
#define tDRPTransition  1 * 10
#define tDRPTry         125 * 10
#define tDRPTryWait     600 * 10
#define tErrorRecovery  25 * 10

#define tVBUSOn         275 * 10    // Max time from entry to Attached.SRC until VBUS reaches minimum vSafe5V
#define tVBUSOff        650 * 10    // Max time from when the sink is detached until the source removes VBUS and reaches vSafe0V
#define tVConnOn        2 * 10      // VConn should be applied prior to VBUS
#define tVConnOnPA      100 * 10    // Max time from when Sink enters PoweredAccessory state until sourcing VCONN
#define tVConnOff       35 * 10     // Max time to remove VCONN supply
#define tSinkAdj        40 * 10     // Nominal time for the sink to reduce its consumption due to a change in Type-C current advertisement

#define tDeviceToggle   3 * 10      // Duration in ms to wait before checking other CC pin for the device
#define tTOG2           30 * 10     //When TOGGLE=1, time at which internal versions of PU_EN1=1 or PU_EN2=1 and PWDN1=PDWN2=0 selected to present externally as a DFP in the DRP toggle

#define T_TIMER_DISABLE (0xFFFF)

typedef enum {
    Source = 0,
    Sink
} SourceOrSink;

// EXTERNS
extern DeviceReg_t              Registers;                                      // Variable holding the current status of the device registers
extern BOOL                     USBPDActive;                                    // Variable to indicate whether the USB PD state machine is active or not
extern BOOL                     USBPDEnabled;                                   // Variable to indicate whether USB PD is enabled (by the host)
extern UINT32                   PRSwapTimer;                                    // Timer used to bail out of a PR_Swap from the Type-C side if necessary
extern USBTypeCPort             PortType;                                       // Variable indicating which type of port we are implementing
extern BOOL                     blnCCPinIsCC1;                                  // Flag to indicate if the CC1 pin has been detected as the CC pin
extern BOOL                     blnCCPinIsCC2;                                  // Flag to indicate if the CC2 pin has been detected as the CC pin
extern BOOL                     blnSMEnabled;                                   // Variable to indicate whether the 300 state machine is enabled
extern ConnectionState          ConnState;                                      // Variable indicating the current Type-C connection state
extern BOOL                     IsHardReset;                                    // Variable indicating that a Hard Reset is occurring
extern BOOL                     PolicyHasContract;                              // Indicates that policy layer has a PD contract

/////////////////////////////////////////////////////////////////////////////
//                            LOCAL PROTOTYPES
/////////////////////////////////////////////////////////////////////////////
VOID TypeCTickAt100us(VOID);
VOID LogTickAt100us(VOID);

VOID InitializeRegisters(VOID);
VOID InitializeTypeCInterrupt(BOOL blnEnable);
VOID InitializeTypeCVariables(VOID);
VOID InitializeTypeC(VOID);
VOID DisableTypeCStateMachine(VOID);
VOID EnableTypeCStateMachine(VOID);
VOID StateMachineTypeC(VOID);
VOID StateMachineDisabled(VOID);
VOID StateMachineErrorRecovery(VOID);
VOID StateMachineDelayUnattached(VOID);
VOID StateMachineUnattached(VOID);
VOID StateMachineAttachWaitSink(VOID);
VOID StateMachineAttachWaitSource(VOID);
VOID StateMachineAttachWaitAccessory(VOID);
VOID StateMachineAttachedSink(VOID);
VOID StateMachineAttachedSource(VOID);
VOID StateMachineTryWaitSink(VOID);
VOID StateMachineTrySource(VOID);
VOID StateMachineDebugAccessory(VOID);
VOID StateMachineAudioAccessory(VOID);
VOID StateMachinePoweredAccessory(VOID);
VOID StateMachineUnsupportedAccessory(VOID);
VOID stateMachineTrySink(VOID);
VOID stateMachineTryWaitSource(VOID);
VOID stateMachineUnattachedSource(VOID);             // Temporary software workaround for entering DRP in Source mode - not part of GUI
VOID SetStateDisabled(VOID);
VOID SetStateErrorRecovery(VOID);
VOID SetStateDelayUnattached(VOID);
VOID SetStateUnattached(VOID);
VOID SetStateAttachWaitSink(VOID);
VOID SetStateAttachWaitSource(VOID);
VOID SetStateAttachWaitAccessory(VOID);
VOID SetStateAttachedSource(VOID);
VOID SetStateAttachedSink(VOID);
VOID RoleSwapToAttachedSink(VOID);
VOID RoleSwapToAttachedSource(VOID);
VOID SetStateTryWaitSink(VOID);
VOID SetStateTrySource(VOID);
VOID SetStateDebugAccessory(VOID);
VOID SetStateAudioAccessory(VOID);
VOID SetStatePoweredAccessory(VOID);
VOID SetStateUnsupportedAccessory(VOID);
VOID SetStateTrySink(VOID);
VOID SetStateTryWaitSource(VOID);
VOID SetStateUnattachedSource(VOID);             // Temporary software workaround for entering DRP in Source mode - not part of GUI
VOID updateSourceCurrent(VOID);
VOID updateSourceMDACHigh(VOID);
VOID updateSourceMDACLow(VOID);
VOID ToggleMeasureCC1(VOID);
VOID ToggleMeasureCC2(VOID);
CCTermType DecodeCCTermination(VOID);
CCTermType DecodeCCTerminationSource(VOID);
CCTermType DecodeCCTerminationSink(VOID);
VOID UpdateSinkCurrent(CCTermType Termination);
VOID ConfigurePortType(UINT8 Control);
VOID UpdateCurrentAdvert(UINT8 Current);
VOID GetDeviceTypeCStatus(UINT8 abytData[]);
UINT8 GetTypeCSMControl(VOID);
UINT8 GetCCTermination(VOID);
BOOL VbusVSafe0V (VOID);
VOID DetectCCPinSource(VOID);
VOID DetectCCPinSink(VOID);
VOID resetDebounceVariables(VOID);
VOID setDebounceVariablesCC1(CCTermType term);
VOID setDebounceVariablesCC2(CCTermType term);
VOID debounceCC(VOID);
VOID debounceSink(VOID);
VOID peekCC1Source(VOID);
VOID peekCC2Source(VOID);
VOID peekCC1Sink(void);
VOID peekCC2Sink(void);
VOID checkForAccessory(VOID);

BOOL GetLocalRegisters(UINT8 * data, INT32 size); //Returns local registers as data array
BOOL GetStateLog(UINT8 * data);   // Returns up to last 12 logs

void setAlternateModes(UINT8 mode);
UINT8 getAlternateModes(void);

VOID ProcessTypeCPDStatus(UINT8* MsgBuffer, UINT8* retBuffer);
VOID ProcessTypeCPDControl(UINT8* MsgBuffer, UINT8* retBuffer);
VOID ProcessLocalRegisterRequest(UINT8* MsgBuffer, UINT8* retBuffer);

VOID ProcessSetTypeCState(UINT8* MsgBuffer, UINT8* retBuffer);
VOID ProcessReadTypeCStateLog(UINT8* MsgBuffer, UINT8* retBuffer);


#endif	/* __FSC_TYPEC_H__ */

