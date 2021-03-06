#include "AlternateModes.h"


void SetStateAlternateUnattached(void)
{
    if ((PortType == USBTypeC_DRP) || ((PortType == USBTypeC_Sink) && (blnAccSupport)))   // If we are a DRP or sink supporting accessories
    {
        SetStateAlternateDRP();
    }
    else if (PortType == USBTypeC_Source)                                       // If we are strictly a Source
    {
        SetStateAlternateUnattachedSource();
    }
    else                                                                        // Otherwise we are a sink
    {
        SetStateAlternateUnattachedSink();
    }
}

void StateMachineAlternateUnattached(void)
{
    if ((PortType == USBTypeC_DRP) || ((PortType == USBTypeC_Sink) && (blnAccSupport)))   // If we are a DRP or sink supporting accessories
    {
        StateMachineAlternateDRP();
    }
    else if (PortType == USBTypeC_Source)                                       // If we are strictly a Source
    {
        StateMachineAlternateUnattachedSource();
    }
    else                                                                        // Otherwise we are a sink
    {
        StateMachineAlternateUnattachedSink();
    }
}

void SetStateAlternateDRP(void)
{
    platform_set_vbus_5v_enable( FALSE );                                       // Disable the 5V output...
    platform_set_vbus_lvl1_enable( FALSE );                                      // Disable the 12V output
    Registers.Switches.byte[0] = 0x00;
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
    while(!VbusVSafe0V());  // waitfor vbus to fall
    ConnState = Unattached;                                         // Set the state machine variable to unattached
    sourceOrSink = Source;                                          // This doesn't make sense for this state, so this will pertain to CC1
    Registers.Switches.byte[0] = 0x46;                              // Enable Pullup1, Pulldown2, and Meas1
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);     // Commit the switch state
    Registers.Power.PWR = 0x7;                                      // Enable everything except internal oscillator
    DeviceWrite(regPower, 1, &Registers.Power.byte);               // Commit the power state
    updateSourceCurrent();                                          // Updates source current
    USBPDDisable(TRUE);                                            // Disable the USB PD state machine (no need to write Device again since we are doing it here)
    SinkCurrent = utccNone;
    resetDebounceVariables();
    blnCCPinIsCC1 = FALSE;                                          // Clear the CC1 pin flag
    blnCCPinIsCC2 = FALSE;                                          // Clear the CC2 pin flag
    StateTimer = tComplianceDRPSwap;                                         //
    tPDDebounce = T_TIMER_DISABLE;                                     // enable the 1st level debounce timer, not used in this state
    tCCDebounce = T_TIMER_DISABLE;                                    // enable the 2nd level debounce timer, not used in this state
    ToggleTimer = tDeviceToggle;                                        // enable the toggle timer
    tOverPDDebounce = tPDDebounceMax;                                  // enable PD filter timer
    WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void StateMachineAlternateDRP(void)
{
    debounceCC();

    if(StateTimer == 0)
    {
        AlternateDRPSwap();
        StateTimer = tComplianceDRPSwap;
    }

    if((CC1TermPrevious >= CCTypeRdUSB) && (CC1TermPrevious <= CCTypeRd3p0) && (sourceOrSink == Source))
    {
        blnCCPinIsCC1 = TRUE;
        blnCCPinIsCC2 = FALSE;
        if ((PortType == USBTypeC_Sink) && (blnAccSupport))             // If we are configured as a sink and support accessories...
            checkForAccessory();                                        // Go to the AttachWaitAcc state
        else                                                            // Otherwise we must be configured as a source or DRP
            SetStateAttachWaitSource();
    }
    // NOTE: Remember sourceOrSink refers to CC1 in this funky state - CC2 is opposite
    else if((CC2TermPrevious >= CCTypeRdUSB) && (CC2TermPrevious <= CCTypeRd3p0) && (sourceOrSink == Sink))
    {
        blnCCPinIsCC1 = FALSE;
        blnCCPinIsCC2 = TRUE;
        if ((PortType == USBTypeC_Sink) && (blnAccSupport))             // If we are configured as a sink and support accessories...
            checkForAccessory();                                        // Go to the AttachWaitAcc state
        else                                                            // Otherwise we must be configured as a source or DRP
            SetStateAttachWaitSource();
    }
    else if((CC1TermPrevious >= CCTypeRdUSB) && (CC1TermPrevious <= CCTypeRd3p0) && (sourceOrSink == Sink))
    {
        blnCCPinIsCC1 = TRUE;
        blnCCPinIsCC2 = FALSE;
        SetStateAttachWaitSink();
    }
    // NOTE: Remember sourceOrSink refers to CC1 in this funky state - CC2 is opposite
    else if((CC2TermPrevious >= CCTypeRdUSB) && (CC2TermPrevious <= CCTypeRd3p0) && (sourceOrSink == Source))
    {
        blnCCPinIsCC1 = FALSE;
        blnCCPinIsCC2 = TRUE;
        SetStateAttachWaitSink();
    }

}

void AlternateDRPSwap(void)
{
    if(sourceOrSink == Source)
    {
        Registers.Switches.byte[0] = 0x89;                              // Enable Pullup2, Pulldown1, and Meas2
        DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);      // Commit the switch state
        sourceOrSink = Sink;
    }
    else
    {
        Registers.Switches.byte[0] = 0x46;                              // Enable Pullup1, Pulldown2, and Meas1
        DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);      // Commit the switch state
        sourceOrSink = Source;
    }
}

void AlternateDRPSourceSinkSwap(void)
{
    if(ConnState == Unattached)
    {
        if(Registers.Switches.MEAS_CC2 == 1)                // CC2 is opposite in this state
        {
            if(sourceOrSink == Source)
            {
                sourceOrSink = Sink;
            }
            else
            {
                sourceOrSink = Source;
            }
        }
    }
}

void SetStateAlternateUnattachedSource(void)
{
    if(PortType == USBTypeC_DRP)
    {
        SetStateAlternateDRP();
        return;
    }
    platform_set_vbus_5v_enable( FALSE );                                       // Disable the 5V output...
    platform_set_vbus_lvl1_enable( FALSE );                                      // Disable the 12V output
    Registers.Switches.byte[0] = 0x00;
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);
    while(!VbusVSafe0V());  // waitfor vbus to fall
    ConnState = UnattachedSource;                                               // Set the state machine variable to unattached
    sourceOrSink = Source;
    Registers.Switches.byte[0] = 0xC4;                                          // Enable both pull-ups and measure on CC1
    Registers.Power.PWR = 0x7;                                                  // Enable everything except internal oscillator
    Registers.Control.HOST_CUR = 0b11;                                          // Set host current to 330uA
    DeviceWrite(regPower, 1, &Registers.Power.byte);                            // Commit the power state
    DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);                    // Commit host current
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);                  // Commit the switch state
    USBPDDisable(TRUE);                                                         // Disable the USB PD state machine (no need to write Device again since we are doing it here)
    SinkCurrent = utccNone;
    resetDebounceVariables();
    blnCCPinIsCC1 = FALSE;                                                      // Clear the CC1 pin flag
    blnCCPinIsCC2 = FALSE;                                                      // Clear the CC2 pin flag
    StateTimer = T_TIMER_DISABLE;                                            // Disable the state timer, not used in this state
    tPDDebounce = tPDDebounceMin;                                               // enable the 1st level debounce timer, not used in this state
    tCCDebounce = tCCDebounceMax;                                               // enable the 2nd level debounce timer, not used in this state
    ToggleTimer = T_TIMER_DISABLE;                                            // disable the toggle timer
    DRPToggleTimer = T_TIMER_DISABLE;                                                      // Timer to switch from unattachedSrc to unattachedSnk in DRP
    tOverPDDebounce = T_TIMER_DISABLE;                                            // enable PD filter timer
    WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void StateMachineAlternateUnattachedSource(void) // CC1 and CC2 are shorted, so we just look for CC1 < 2.6V for Ra/Rd and > 0.2V for no Ra
{
    debounceCC();

    if(blnAccSupport)
    {
        if (CC1TermPrevious == CCTypeRa)
        {
            blnCCPinIsCC1 = FALSE;                                                  // Setting both to false will have the next state figure it out
            blnCCPinIsCC2 = FALSE;
            SetStateAttachWaitSource();
        }
    }

    if ((CC1TermPrevious >= CCTypeRdUSB) && (CC1TermPrevious < CCTypeUndefined))    // If the CC1 pin is Rd
    {
        blnCCPinIsCC1 = FALSE;                                                  // Setting both to false will have the next state figure it out
        blnCCPinIsCC2 = FALSE;
        SetStateAttachWaitSource();                                             // Go to the Attached.Src state
    }
}

void StateMachineAlternateUnattachedSink(void)
{
    debounceCC();

    if ((CC1TermPrevious >= CCTypeRdUSB) && (CC1TermPrevious < CCTypeUndefined) && ((CC2TermPrevious == CCTypeRa) || CC2TermPrevious == CCTypeOpen))    // If the CC1 pin is Rd for atleast tPDDebounce...
    {
        blnCCPinIsCC1 = TRUE;                                                   // The CC pin is CC1
        blnCCPinIsCC2 = FALSE;
        SetStateAttachWaitSink();                                                  // Go to the Attached.Snk state
    }
    else if ((CC2TermPrevious >= CCTypeRdUSB) && (CC2TermPrevious < CCTypeUndefined) && ((CC1TermPrevious == CCTypeRa) || CC1TermPrevious == CCTypeOpen))   // If the CC2 pin is Rd for atleast tPDDebounce...
    {
        blnCCPinIsCC1 = FALSE;                                                  // The CC pin is CC2
        blnCCPinIsCC2 = TRUE;
        SetStateAttachWaitSink();                                                  // Go to the Attached.Snk state
    }
}

void SetStateAlternateUnattachedSink(void)
{
    platform_set_vbus_5v_enable( FALSE );                                       // Disable the 5V output...
    platform_set_vbus_lvl1_enable( FALSE );                                      // Disable the 12V output
    Registers.Switches.byte[0] = 0x00;
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]); // Disable the 12V output
    while(!VbusVSafe0V());  // waitfor vbus to fall

    ConnState = Unattached;                                                     // Set the state machine variable to unattached
    sourceOrSink = Sink;
    Registers.Switches.byte[0] = 0x07;                                          // Enable both pull-downs and measure on CC1
    Registers.Power.PWR = 0x7;                                                  // Enable everything except internal oscillator
    DeviceWrite(regPower, 1, &Registers.Power.byte);                            // Commit the power state
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);                  // Commit the switch state
    USBPDDisable(TRUE);                                                         // Disable the USB PD state machine (no need to write Device again since we are doing it here)
    SinkCurrent = utccNone;
    resetDebounceVariables();
    blnCCPinIsCC1 = FALSE;                                                      // Clear the CC1 pin flag
    blnCCPinIsCC2 = FALSE;                                                      // Clear the CC2 pin flag
    StateTimer = T_TIMER_DISABLE;                                             // Disable the state timer, not used in this state
    tPDDebounce = tPDDebounceMin;                                               // enable the 1st level debounce timer, not used in this state
    tCCDebounce = tCCDebounceMax;                                               // enable the 2nd level debounce timer, not used in this state
    ToggleTimer = T_TIMER_DISABLE;                                            // disable the toggle timer
    DRPToggleTimer = tDeviceToggle;                                             // Timer to switch from unattachedSrc to unattachedSnk in DRP
    tOverPDDebounce = T_TIMER_DISABLE;                                        // enable PD filter timer
    WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

void SetStateAlternateAudioAccessory(void)
{
    platform_set_vbus_5v_enable( FALSE );                                       // Disable the 5V output...
    platform_set_vbus_lvl1_enable( FALSE );                                      // Disable the 12V output
    ConnState = AudioAccessory;                                                 // Set the state machine variable to Audio.Accessory
    sourceOrSink = Source;
    Registers.Power.PWR = 0x7;                                                  // Enable everything except internal oscillator
    DeviceWrite(regPower, 1, &Registers.Power.byte);                            // Commit the power state
    Registers.Control.HOST_CUR = 0b11;                                          // Set host current to 330uA
    DeviceWrite(regControl0, 1, &Registers.Control.byte[0]);                    // Commit host current
    Registers.Switches.byte[0] = 0xC4;                                          // Enable both pull-ups and measure on CC1
    DeviceWrite(regSwitches0, 1, &Registers.Switches.byte[0]);                  // Commit the switch state
    SinkCurrent = utccNone;                                                     // Not used in accessories
    tOverPDDebounce = T_TIMER_DISABLE;                                       // Disable PD filter timer
    StateTimer = T_TIMER_DISABLE;                                            // Disable the state timer, not used in this state
    tPDDebounce = tCCDebounceNom;                                               // Once in this state, we are waiting for the lines to be stable for tCCDebounce before changing states
    tCCDebounce = T_TIMER_DISABLE;                                           // Disable the 2nd level debouncing initially to force completion of a 1st level debouncing
    ToggleTimer = T_TIMER_DISABLE;                                           // Once we are in the audio.accessory state, we are going to stop toggling and only monitor CC1
    WriteStateLog(&TypeCStateLog, ConnState, Timer_tms, Timer_S);
}

CCTermType AlternateDecodeCCTerminationSource(void)
{
    CCTermType Termination = CCTypeUndefined;            // By default set it to undefined

    Registers.Measure.MDAC = MDAC_2P05V;                                         // Set up DAC threshold to 2.6V
    DeviceWrite(regMeasure, 1, &Registers.Measure.byte);                        // Commit the DAC threshold
    platform_delay_10us(25);                                                    // Delay to allow measurement to settle
    DeviceRead(regStatus0, 1, &Registers.Status.byte[4]);

    if (Registers.Status.COMP == 1)
    {
        Termination = CCTypeOpen;
        return Termination;
    }
    else if(Registers.Status.BC_LVL == 0)
    {
        Termination = CCTypeRa;
    }
    else
    {
        Termination = CCTypeRdUSB;
    }
    return Termination;                             // Return the termination type
}

