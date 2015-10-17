#include "fusb30x_global.h"
#include "GenericTypeDefs.h"
#include "platform_helpers.h"
#include "core.h"
#include "hostcomm.h"

// ##### Packet Structure ##### //
#define PKTOUT_REQUEST                  0x00
#define PKTOUT_VERSION                  0x01
#define PKTIN_REQUEST                   0x00
#define PKTIN_STATUS                    0x01
#define PKTIN_ERRORCODE                 0x03

// ##### Command definitions ##### //
#define CMD_GETDEVICEINFO               0x00
#define CMD_USBPD_BUFFER_READ           0xA0
#define CMD_USBPD_STATUS                0xA2
#define CMD_USBPD_CONTROL               0xA3
#define CMD_GET_SRC_CAPS                0xA4
#define CMD_GET_SINK_CAPS               0xA5
#define CMD_GET_SINK_REQ                0xA6
#define CMD_ENABLE_PD                   0xA7
#define CMD_DISABLE_PD                  0xA8
#define CMD_GET_ALT_MODES               0xA9
#define CMD_GET_MANUAL_RETRIES          0xAA
#define CMD_SET_STATE_UNATTACHED        0xAB
#define CMD_ENABLE_TYPEC_SM             0xAC
#define CMD_DISABLE_TYPEC_SM            0xAD
#define CMD_SEND_HARD_RESET             0xAE

#define CMD_DEVICE_LOCAL_REGISTER_READ  0xB0    // xBX designation used for automated testing
#define CMD_SET_STATE                   0xB1
#define CMD_READ_STATE_LOG              0xB2
#define CMD_READ_PD_STATE_LOG           0xB3

#define CMD_READ_I2C                    0xC0
#define CMD_WRITE_I2C                   0xC1
#define CMD_GET_VBUS5V                  0xC4
#define CMD_SET_VBUS5V                  0xC5
#define CMD_GET_INTN                    0xC6

#ifdef DEBUG
#define CMD_GET_TIMER_TICKS             0xF0
#define CMD_GET_SM_TICKS                0xF1
#define CMD_GET_GPIO_SM_TOGGLE          0xF2
#define CMD_SET_GPIO_SM_TOGGLE          0xF3
#endif  // DEBUG

#define TEST_FIRMWARE                   0X01    // For GUI identification of firmware

/* Device Info */

/* MCU Identification */
enum {
    mcuUnknown		= 0,
    mcuPIC18F14K50	= 1,
    mcuPIC32MX795F512L	= 2,
    mcuPIC32MX250F128B	= 3,
    mcuGENERIC_LINUX	= 4,	// Linux driver
} mcu_t;

/* Device Type Identification */
enum {
    dtUnknown		= -1,
    dtUSBI2CStandard	= 0,
    dtUSBI2CPDTypeC	= 1
};

/* Board Configuration */
enum {
    bcUnknown		= -1,
    bcStandardI2CConfig	= 0,
    bcFUSB300Eval	= 0x100,
    bcFUSB302FPGA	= 0x200,
    bcFM14014		= 0x300
};

#define MY_MCU          mcuGENERIC_LINUX
#define MY_DEV_TYPE     dtUSBI2CPDTypeC
#define MY_BC           bcStandardI2CConfig

static void fusb_hc_Handle_I2CRead(u8* inBuf, u8* outBuf);
static void fusb_hc_Handle_I2CWrite(u8* inBuf, u8* outBuf);
static void fusb_hc_GetVBus5V(u8* outMsgBuffer);
static void fusb_hc_SetVBus5V(u8* inMsgBuffer, u8* outMsgBuffer);
static void fusb_hc_GetIntN(u8* outMsgBuffer);
#ifdef DEBUG
static void fusb_hc_GetTimerTicks(u8* outMsgBuffer);
static void fusb_hc_GetSMTicks(u8* outMsgBuffer);
static void fusb_hc_GetGPIO_SM_Toggle(u8* outMsgBuffer);
static void fusb_hc_SetGPIO_SM_Toggle(u8* inMsgBuffer, u8* outMsgBuffer);
#endif

/*******************************************************************************
* Function:        fusb_ProcessMsg
* Input:           inMsgBuffer: Input array to process
*                  outMsgBuffer: Result is stored here
* Return:          none
* Description:     Processes the input array as a command with optional data.
*                  Provides a debugging interface to the caller.
********************************************************************************/
void fusb_ProcessMsg(u8* inMsgBuffer, u8* outMsgBuffer)
{
    u8 i;

    outMsgBuffer[0] = inMsgBuffer[0];           // Echo the request

    switch (inMsgBuffer[0])
    {
        case CMD_GETDEVICEINFO:
            if (inMsgBuffer[1] != 0)
                outMsgBuffer[1] = 0x01;                                     // Return that the version is not recognized
            else
            {
                outMsgBuffer[1] = 0x00;                                     // Return that the command was recognized
                outMsgBuffer[4] = MY_MCU;                                   // MCU
                outMsgBuffer[5] = MY_DEV_TYPE;                              // USB to I2C device with USB PD and Type C

                outMsgBuffer[6] = core_get_rev_lower();
                outMsgBuffer[7] = core_get_rev_upper();

                outMsgBuffer[8] = 0xFF & MY_BC;                             // [9:8] make up the board configuration used
                outMsgBuffer[9] = 0xFF & (MY_BC >> 8);

                for (i = 0; i < 16; i++)                                    // Loop through all the serial number bytes
                {
                    outMsgBuffer[i + 10] = 0x00; //Read_b_eep(i);           // return each of the bytes from EEPROM data
                }

                outMsgBuffer[26] = TEST_FIRMWARE;                           // Reports if this firmware runs the test suite
            }
            break;
        case CMD_USBPD_BUFFER_READ:
            core_process_pd_buffer_read(inMsgBuffer, outMsgBuffer);         // Grab as many bytes from the USB PD buffer as possible
            break;
        case CMD_USBPD_STATUS:
            core_process_typec_pd_status(inMsgBuffer, outMsgBuffer);        // Get the status of the PD state machines
            break;
        case CMD_USBPD_CONTROL:
            core_process_typec_pd_control(inMsgBuffer, outMsgBuffer);       // Update the PD state machine settings
            break;
        case CMD_GET_SRC_CAPS:
            core_get_source_caps(outMsgBuffer);
            break;
        case CMD_GET_SINK_CAPS:
            core_get_sink_caps(outMsgBuffer);
            break;
        case CMD_GET_SINK_REQ:
            core_get_sink_req(outMsgBuffer);
            break;
        case CMD_ENABLE_PD:
            core_enable_pd(TRUE);
            outMsgBuffer[0] = CMD_ENABLE_PD;
            outMsgBuffer[1] = 1;
            break;
        case CMD_DISABLE_PD:
            core_enable_pd(FALSE);
            outMsgBuffer[0] = CMD_DISABLE_PD;
            outMsgBuffer[1] = 0;
            break;
        case CMD_GET_ALT_MODES:
            outMsgBuffer[0] = CMD_GET_ALT_MODES;                            // Success
            outMsgBuffer[1] = core_get_alternate_modes();                   // Value
            break;
        case CMD_GET_MANUAL_RETRIES:
            outMsgBuffer[0] = CMD_GET_MANUAL_RETRIES;
            outMsgBuffer[1] = core_get_manual_retries();
            break;
        case CMD_SET_STATE_UNATTACHED:
            core_set_state_unattached();
            outMsgBuffer[0] = CMD_SET_STATE_UNATTACHED;
            outMsgBuffer[1] = 0;
            break;
        case CMD_ENABLE_TYPEC_SM:
            core_enable_typec(TRUE);
            outMsgBuffer[0] = CMD_ENABLE_TYPEC_SM;
            outMsgBuffer[1] = 0;
            break;
        case CMD_DISABLE_TYPEC_SM:
            core_enable_typec(FALSE);
            outMsgBuffer[0] = CMD_DISABLE_TYPEC_SM;
            outMsgBuffer[1] = 0;
            break;
        case CMD_DEVICE_LOCAL_REGISTER_READ:
            core_process_local_register_request(inMsgBuffer, outMsgBuffer); // Send local register values
            break;
        case CMD_SET_STATE:                                                 // Set state
            core_process_set_typec_state(inMsgBuffer, outMsgBuffer);
            break;
        case CMD_READ_STATE_LOG:                                            // Read state log
            core_process_read_typec_state_log(inMsgBuffer, outMsgBuffer);
            break;
        case CMD_READ_PD_STATE_LOG:                                         // Read PD state log
            core_process_read_pd_state_log(inMsgBuffer, outMsgBuffer);
            break;
        case CMD_READ_I2C:
            fusb_hc_Handle_I2CRead(inMsgBuffer, outMsgBuffer);
            break;
        case CMD_WRITE_I2C:
            fusb_hc_Handle_I2CWrite(inMsgBuffer, outMsgBuffer);
            break;
        case CMD_GET_VBUS5V:
            fusb_hc_GetVBus5V(outMsgBuffer);
            break;
        case CMD_SET_VBUS5V:
            fusb_hc_SetVBus5V(inMsgBuffer, outMsgBuffer);
            break;
        case CMD_GET_INTN:
            fusb_hc_GetIntN(outMsgBuffer);
            break;
        case CMD_SEND_HARD_RESET:
            core_send_hard_reset();
            outMsgBuffer[0] = CMD_SEND_HARD_RESET;
            outMsgBuffer[1] = 0;
            break;
#ifdef DEBUG
        case CMD_GET_TIMER_TICKS:
            fusb_hc_GetTimerTicks(outMsgBuffer);
            break;
        case CMD_GET_SM_TICKS:
            fusb_hc_GetSMTicks(outMsgBuffer);
            break;
        case CMD_GET_GPIO_SM_TOGGLE:
            fusb_hc_GetGPIO_SM_Toggle(outMsgBuffer);
            break;
        case CMD_SET_GPIO_SM_TOGGLE:
            fusb_hc_SetGPIO_SM_Toggle(inMsgBuffer, outMsgBuffer);
            break;
#endif  // DEBUG
        default:
            outMsgBuffer[1] = 0x01;                                         // Return that the request is not implemented
            break;
    }
}

/*******************************************************************************
* Function:        fusb_hc_Handle_I2CRead
* Input:           inBuf: Input buffer to parse. inBuf[0]: Addr, other bytes ignored
*                  outBuf: I2C read result stored in outBuf[0]
* Return:          none
* Description:     Read an unsigned byte from the I2C peripheral
********************************************************************************/
static void fusb_hc_Handle_I2CRead(u8* inBuf, u8* outBuf)
{
    if (!fusb_I2C_ReadData(inBuf[1], outBuf))
    {
        printk(KERN_ERR "FUSB  %s - Error: Could not read I2C Data!\n", __func__);
    }
}

/*******************************************************************************
* Function:        fusb_hc_Handle_I2CWrite
* Input:           inBuf: Input buffer to parse.
*                       inBuf[1]: Start of address range to write to
*                       inBuf[2]: Num bytes to write
*                       inBuf[3.. inBuf[1]]: Data to write
*                  outBuf: Output buffer populated with result:
*                       Success: outBuf[0] = 1
*                       Failure: outBuf[0] = 0
* Return:          none
* Description:     Write an unsigned byte to the I2C peripheral
********************************************************************************/
static void fusb_hc_Handle_I2CWrite(u8* inBuf, u8* outBuf)
{
    if (!fusb_I2C_WriteData(inBuf[1], inBuf[2], &inBuf[3]))
    {
        printk(KERN_ERR "FUSB  %s - Error: Could not write I2C Data!\n", __func__);
        outBuf[0] = 0;  // Notify failure
    }
    else
    {
        outBuf[0] = 1;  // Notify success
    }
}

/*******************************************************************************
* Function:        fusb_hc_GetVBus5V
* Input:           none
*                  outBuf: Output buffer populated with result:
*                       Success:    outBuf[0] = 1
*                                   outBuf[1] = value
*                       Failure:    outBuf[0] = 0
* Return:          none
* Description:     Get the value of VBus 5V
********************************************************************************/
static void fusb_hc_GetVBus5V(u8* outMsgBuffer)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(KERN_ERR "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        outMsgBuffer[0] = 0;
        return;
    }

    outMsgBuffer[0] = 1;
    outMsgBuffer[1] = fusb_GPIO_Get_VBus5v() ? 1 : 0;
}

/*******************************************************************************
* Function:        fusb_hc_SetVBus5V
* Input:           inMsgBuffer:     inMsgBuffer[1] = [1 || 0], value to write
*                  outBuf: Output buffer populated with result:
*                       Success:    outBuf[0] = 1
*                                   outBuf[1] = value that was set
*                       Failure:    outBuf[0] = 0
* Return:          none
* Description:     Set the value of VBus 5V
********************************************************************************/
static void fusb_hc_SetVBus5V(u8* inMsgBuffer, u8* outMsgBuffer)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(KERN_ERR "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        outMsgBuffer[0] = 0;
        return;
    }

    fusb_GPIO_Set_VBus5v(inMsgBuffer[1]);
    outMsgBuffer[0] = 1;
    outMsgBuffer[1] = fusb_GPIO_Get_VBus5v() ? 1 : 0;
}

/*******************************************************************************
* Function:        fusb_hc_GetIntN
* Input:           none
*                  outBuf: Output buffer populated with result:
*                       Success:    outBuf[0] = 1
*                                   outBuf[1] = value
*                       Failure:    outBuf[0] = 0
* Return:          none
* Description:     Get the value of Int_N
********************************************************************************/
static void fusb_hc_GetIntN(u8* outMsgBuffer)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(KERN_ERR "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        outMsgBuffer[0] = 0;
        return;
    }

    outMsgBuffer[0] = 1;
    outMsgBuffer[1] = fusb_GPIO_Get_IntN() ? 1 : 0;
}

#ifdef DEBUG
/*******************************************************************************
* Function:        fusb_hc_GetTimerTicks
* Input:           none
*                  outBuf: Output buffer populated with result:
*                       outBuf[0]: 1 on success, 0 on failure
*                       outBuf[1]: Number of timer ticks passed
*                       outBuf[2]: Number of times the timer tick ticker rolled over
* Return:          none
* Description:     DEBUG: Get the number of timer ticks that have passed
********************************************************************************/
static void fusb_hc_GetTimerTicks(u8* outMsgBuffer)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        outMsgBuffer[0] = 0;
        return;
    }

    outMsgBuffer[0] = 1;    // Success
    outMsgBuffer[1] = chip->dbgTimerTicks;
    outMsgBuffer[2] = chip->dbgTimerRollovers;
}

/*******************************************************************************
* Function:        fusb_hc_GetSMTicks
* Input:           none
*                  outBuf: Output buffer populated with result:
*                       outBuf[0]: 1 on success, 0 on failure
*                       outBuf[1]: Number of SM ticks passed
*                       outBuf[2]: Number of times the SM tick ticker rolled over
* Return:          none
* Description:     DEBUG: Get the number of SM ticks that have passed
********************************************************************************/
static void fusb_hc_GetSMTicks(u8* outMsgBuffer)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        outMsgBuffer[0] = 0;
        return;
    }

    outMsgBuffer[0] = 1;    // Success
    outMsgBuffer[1] = chip->dbgSMTicks;
    outMsgBuffer[2] = chip->dbgSMRollovers;
}

/*******************************************************************************
* Function:        fusb_hc_GetGPIO_SM_Toggle
* Input:           none
*                  outBuf: Output buffer populated with result:
*                       Success:    outBuf[0] = 1
*                                   outBuf[1] = value
*                       Failure:    outBuf[0] = 0
* Return:          none
* Description:     Get the value of the SM toggle GPIO
********************************************************************************/
static void fusb_hc_GetGPIO_SM_Toggle(u8* outMsgBuffer)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(KERN_ERR "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        outMsgBuffer[0] = 0;
        return;
    }

    outMsgBuffer[0] = 1;
    outMsgBuffer[1] = dbg_fusb_GPIO_Get_SM_Toggle() ? 1 : 0;
}

/*******************************************************************************
* Function:        fusb_hc_SetGPIO_SM_Toggle
* Input:           inMsgBuffer:     inMsgBuffer[1] = [1 || 0], value to write
*                  outBuf: Output buffer populated with result:
*                       Success:    outBuf[0] = 1
*                                   outBuf[1] = value that was set
*                       Failure:    outBuf[0] = 0
* Return:          none
* Description:     Set the value of the SM toggle GPIO
********************************************************************************/
static void fusb_hc_SetGPIO_SM_Toggle(u8* inMsgBuffer, u8* outMsgBuffer)
{
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        printk(KERN_ERR "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        outMsgBuffer[0] = 0;
        return;
    }

    dbg_fusb_GPIO_Set_SM_Toggle(inMsgBuffer[1]);
    outMsgBuffer[0] = 1;
    outMsgBuffer[1] = dbg_fusb_GPIO_Get_SM_Toggle() ? 1 : 0;
}
#endif  // DEBUG
