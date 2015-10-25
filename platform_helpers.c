#include <linux/kernel.h>
#include <linux/stat.h>                                                         // File permission masks
#include <linux/types.h>                                                        // Kernel datatypes
#include <linux/i2c.h>                                                          // I2C access, mutex
#include <linux/errno.h>                                                        // Linux kernel error definitions
#include <linux/hrtimer.h>                                                      // hrtimer
#include <linux/workqueue.h>                                                    // work_struct, delayed_work
#include <linux/delay.h>                                                        // udelay, usleep_range, msleep
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "fusb30x_global.h"                                                     // Chip structure access
#include "core.h"                                                          // Core access
#include "hostcomm.h"
#include "platform_helpers.h"

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/********************************************        GPIO Interface         ******************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/

#define FUSB_DT_GPIO_INTN               "fairchild,int_n"           // Name of the Int_N GPIO pin in the Device Tree
#define FUSB_DT_GPIO_VBUS_5V            "fairchild,vbus5v"          // Name of the VBus 5V GPIO pin in the Device Tree
#define FUSB_DT_GPIO_VBUS_OTHER         "fairchild,vbusOther"       // Name of the VBus Other GPIO pin in the Device Tree

#ifdef DEBUG
#define FUSB_DT_GPIO_DEBUG_SM_TOGGLE    "fairchild,dbg_sm"          // Name of the debug State Machine toggle GPIO pin in the Device Tree
#endif  // DEBUG

int fusb_InitializeGPIO(void)
{
	int ret = 0;
	struct device_node* node;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return -ENOMEM;
	}
	/* Get our device tree node */
	node = chip->client->dev.of_node;

	/* Get our GPIO pins from the device tree, and then set their direction (input/output) */
	chip->gpio_IntN = of_get_named_gpio(node, FUSB_DT_GPIO_INTN, 0);
	if (chip->gpio_IntN < 0 || gpio_is_valid(chip->gpio_IntN) < 0) {
		dev_err(&chip->client->dev, "%s - Error: Could not get GPIO for Int_N! Error code: %d\n", __func__, chip->gpio_IntN);
		return chip->gpio_IntN;
	}
	ret = gpio_direction_input(chip->gpio_IntN);
	if (ret != 0) {
		dev_err(&chip->client->dev, "%s - Error: Could not set GPIO direction to input for Int_N! Error code: %d\n", __func__, ret);
		return ret;
	}

	// Export to sysfs
	gpio_export(chip->gpio_IntN, false);
	gpio_export_link(&chip->client->dev, FUSB_DT_GPIO_INTN, chip->gpio_IntN);

	// VBus 5V
	chip->gpio_VBus5V = of_get_named_gpio(node, FUSB_DT_GPIO_VBUS_5V, 0);
	if (chip->gpio_VBus5V < 0 || gpio_is_valid(chip->gpio_VBus5V) < 0) {
		dev_err(&chip->client->dev, "%s - Error: Could not get GPIO for VBus5V! Error code: %d\n", __func__, chip->gpio_VBus5V);
		return chip->gpio_VBus5V;
	}
	ret = gpio_direction_output(chip->gpio_VBus5V, chip->gpio_VBus5V_value);
	if (ret != 0) {
		dev_err(&chip->client->dev, "%s - Error: Could not set GPIO direction to output for VBus5V! Error code: %d\n", __func__, ret);
		return ret;
	}

	// Export to sysfs
	gpio_export(chip->gpio_VBus5V, false);
	gpio_export_link(&chip->client->dev, FUSB_DT_GPIO_VBUS_5V, chip->gpio_VBus5V);

	printk(KERN_DEBUG "FUSB  %s - VBus 5V initialized as pin '%d' and is set to '%d'\n", __func__, chip->gpio_VBus5V, chip->gpio_VBus5V_value ? 1 : 0);

	// VBus other (eg. 12V)
	// NOTE - This VBus is optional, so if it doesn't exist then fake it like it's on.
	chip->gpio_VBusOther = of_get_named_gpio(node, FUSB_DT_GPIO_VBUS_OTHER, 0);
	if (chip->gpio_VBusOther < 0 || gpio_is_valid(chip->gpio_VBusOther) < 0) {
		// Soft fail - provide a warning, but don't quit because we don't really need this VBus if only using VBus5v
		printk(KERN_WARNING "%s - Error: Could not get GPIO for VBusOther! Error code: %d\n*** Driver will load without this feature ***\n", __func__, chip->gpio_VBusOther);
	} else {
		ret = gpio_direction_output(chip->gpio_VBusOther, chip->gpio_VBusOther_value);
		if (ret != 0) {
			dev_err(&chip->client->dev, "%s - Error: Could not set GPIO direction to output for VBus5V! Error code: %d\n", __func__, ret);
			return ret;
		}
	}

#ifdef DEBUG
	// State Machine Debug Notification
	// Optional GPIO - toggles each time the state machine is called
	chip->dbg_gpio_StateMachine = of_get_named_gpio(node, FUSB_DT_GPIO_DEBUG_SM_TOGGLE, 0);
	if (chip->dbg_gpio_StateMachine < 0 || gpio_is_valid(chip->dbg_gpio_StateMachine) < 0) {
	// Soft fail - provide a warning, but don't quit because we don't really need this VBus if only using VBus5v
		printk(KERN_WARNING "%s - Error: Could not get GPIO for VBusOther! Error code: %d\n*** Driver will load without this feature ***\n", __func__, chip->dbg_gpio_StateMachine);
	} else {
		ret = gpio_direction_output(chip->dbg_gpio_StateMachine, chip->dbg_gpio_StateMachine_value);
		if (ret != 0) {
			dev_err(&chip->client->dev, "%s - Error: Could not set GPIO direction to output for SM Debug Toggle! Error code: %d\n", __func__, ret);
			return ret;
		}

		// Export to sysfs
		gpio_export(chip->dbg_gpio_StateMachine, false);
		gpio_export_link(&chip->client->dev, FUSB_DT_GPIO_DEBUG_SM_TOGGLE, chip->dbg_gpio_StateMachine);
	}
#endif  // DEBUG

	return 0;   // Success!
}

void fusb_GPIO_Set_VBus5v(bool set)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
	}
	gpio_set_value(chip->gpio_VBus5V, set ? 1 : 0);
	chip->gpio_VBus5V_value = set;

	printk(KERN_DEBUG "FUSB  %s - VBus 5V set to: %d\n", __func__, chip->gpio_VBus5V_value ? 1 : 0);
}

void fusb_GPIO_Set_VBusOther(bool set)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
	}

	// Only try to set if feature is enabled, otherwise just fake it
	if (chip->gpio_VBusOther >= 0) {
		gpio_set_value(chip->gpio_VBusOther, set ? 1 : 0);
	}

	chip->gpio_VBusOther_value = set;
}

bool fusb_GPIO_Get_VBus5v(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();

	if (!chip) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	}
	if (chip->gpio_VBus5V < 0) {
		printk(KERN_DEBUG "FUSB  %s - Error: VBus 5V pin invalid! Pin value: %d\n", __func__, chip->gpio_VBus5V);
	}
	return chip->gpio_VBus5V_value;
}

bool fusb_GPIO_Get_VBusOther(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();

	if (!chip) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	}

	return chip->gpio_VBusOther_value;
}

bool fusb_GPIO_Get_IntN(void)
{
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();

	if (!chip) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	} else {
		ret = !gpio_get_value(chip->gpio_IntN); // Int_N is active low
		if (ret < 0) {
			dev_err(&chip->client->dev, "%s - Error: Could not get GPIO value for gpio_IntN! Error code: %d\n", __func__, ret);
			return false;
		}
		return (ret > 0);
	}
}

#ifdef DEBUG
void dbg_fusb_GPIO_Set_SM_Toggle(bool set)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();

	if (!chip) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
	}
	gpio_set_value(chip->dbg_gpio_StateMachine, set ? 1 : 0);
	chip->dbg_gpio_StateMachine_value = set;

	printk(KERN_DEBUG "FUSB  %s - State machine toggle GPIO set to: %d\n", __func__, chip->dbg_gpio_StateMachine_value ? 1 : 0);
}

bool dbg_fusb_GPIO_Get_SM_Toggle(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return false;
	}
	if (chip->dbg_gpio_StateMachine < 0) {
		printk(KERN_DEBUG "FUSB  %s - Error: State machine toggle debug pin invalid! Pin number: %d\n", __func__, chip->dbg_gpio_StateMachine);
	}
	return chip->dbg_gpio_StateMachine_value;
}
#endif  // DEBUG

void fusb_GPIO_Cleanup(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	if (gpio_is_valid(chip->gpio_IntN) >= 0) {
		gpio_unexport(chip->gpio_IntN);
		gpio_free(chip->gpio_IntN);
	}

	if (gpio_is_valid(chip->gpio_VBus5V) >= 0) {
		gpio_unexport(chip->gpio_VBus5V);
		gpio_free(chip->gpio_VBus5V);
	}

	if (gpio_is_valid(chip->gpio_VBusOther) >= 0) {
		gpio_free(chip->gpio_VBusOther);
	}

#ifdef DEBUG
	if (gpio_is_valid(chip->dbg_gpio_StateMachine) >= 0) {
		gpio_unexport(chip->dbg_gpio_StateMachine);
		gpio_free(chip->dbg_gpio_StateMachine);
	}
#endif  // DEBUG
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/********************************************         I2C Interface         ******************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
bool fusb_I2C_WriteData(unsigned char address, unsigned char length, unsigned char* data)
{
	int i = 0;
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();

	// Sanity check
	if (chip == NULL || chip->client == NULL || data == NULL) {
		printk(KERN_ERR "%s - Error: %s is NULL!\n", __func__, (chip == NULL ? "Internal chip structure"
		    : (chip->client == NULL ? "I2C Client"
		    : "Write data buffer")));
		return false;
	}

	mutex_lock(&chip->lock);
	// Retry on failure up to the retry limit
	for (i = 0; i <= chip->numRetriesI2C; i++) {
		ret = i2c_smbus_write_i2c_block_data(chip->client,                      // Perform the actual I2C write on our client
					     address,                           // Register address to write to
					     length,                            // Number of bytes to write
					     data);                             // Ptr to unsigned char data
		if (ret < 0) {                                                          // Errors report as negative
			if (ret == ERANGE) {
				dev_err(&chip->client->dev,
					"%s - I2C Error writing byte data. Address: '0x%02x', Return: -ERANGE.  Attempt #%d / %d...\n",
					__func__, address, i, chip->numRetriesI2C);
			} else if (ret == EINVAL) {
				dev_err(&chip->client->dev,
					"%s - I2C Error writing byte data. Address: '0x%02x', Return: -EINVAL.  Attempt #%d / %d...\n",
					__func__, address, i, chip->numRetriesI2C);
			} else {
				dev_err(&chip->client->dev,
					"%s - Unexpected I2C error writing byte data. Address: '0x%02x', Return: '%04x'.  Attempt #%d / %d...\n",
					__func__, address, ret, i, chip->numRetriesI2C);
			}
		} else {
			// Successful i2c writes should always return 0
			{
				break;
			}
		}
	}
	mutex_unlock(&chip->lock);
	return (ret >= 0);
}

bool fusb_I2C_ReadData(unsigned char address, unsigned char* data)
{
	int i = 0;
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();

	if (chip == NULL || chip->client == NULL || data == NULL) {
		printk(KERN_ERR "%s - Error: %s is NULL!\n", __func__, (chip == NULL ? "Internal chip structure"
			: (chip->client == NULL ? "I2C Client"
			: "read data buffer")));
		return false;
	}
	mutex_lock(&chip->lock);
	// Retry on failure up to the retry limit
	for (i = 0; i <= chip->numRetriesI2C; i++) {
		// Read a byte of data from address
		ret = i2c_smbus_read_byte_data(chip->client, (u8)address);
			// Errors report as negative
		if (ret < 0) {
			if (ret == ERANGE) {
				dev_err(&chip->client->dev, "%s - I2C Error reading byte data. Address: '0x%02x', Return: -ERANGE.  Attempt #%d / %d...\n", __func__, address, i, chip->numRetriesI2C);
			} else if (ret == EINVAL) {
				dev_err(&chip->client->dev, "%s - I2C Error reading byte data. Address: '0x%02x', Return: -EINVAL.  Attempt #%d / %d...\n", __func__, address, i, chip->numRetriesI2C);
			} else {
				dev_err(&chip->client->dev, "%s - Unexpected I2C error reading byte data. Address: '0x%02x', Return: '%04x'.  Attempt #%d / %d...\n", __func__, address, ret, i, chip->numRetriesI2C);
			}
		} else {
			// Successful i2c writes should always return 0
			*data = (unsigned char)ret;
			break;
		}
	}
	mutex_unlock(&chip->lock);
	return (ret >= 0);
}


/*********************************************************************************************************************/
/*********************************************************************************************************************/
/********************************************        Timer Interface        ******************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
static const unsigned long g_fusb_timer_tick_period_ns = 100000;    // Tick SM every 100us -> 100000ns

/*******************************************************************************
* Function:        _fusb_TimerHandler
* Input:           timer: hrtimer struct to be handled
* Return:          HRTIMER_RESTART to restart the timer, or HRTIMER_NORESTART otherwise
* Description:     Ticks state machine timer counters and rearms itself
********************************************************************************/
enum hrtimer_restart _fusb_TimerHandler(struct hrtimer* timer)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return HRTIMER_NORESTART;
	}

	if (!timer) {
		printk(KERN_ALERT "FUSB  %s - Error: High-resolution timer is NULL!\n", __func__);
		return HRTIMER_NORESTART;
	}

	core_tick_at_100us();

#ifdef DEBUG
	if (chip->dbgTimerTicks++ >= U8_MAX) {
		chip->dbgTimerRollovers++;
	}
#endif  // DEBUG

	// Reset the timer expiration
	hrtimer_forward(timer, ktime_get(), ktime_set(0, g_fusb_timer_tick_period_ns));

	return HRTIMER_RESTART;         // Requeue the timer
}

void fusb_InitializeTimer(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n",
		       __func__);
		return;
	}

	// Init the timer structure
	hrtimer_init(&chip->timer_state_machine, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	// Assign the callback to call when time runs out
	chip->timer_state_machine.function = _fusb_TimerHandler;

	printk(KERN_DEBUG "FUSB  %s - Timer initialized!\n", __func__);
}

void fusb_StartTimers(void)
{
	ktime_t ktime;
	struct fusb30x_chip* chip;

	chip = fusb30x_GetChip();
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n",
		       __func__);
		return;
	}

	// Convert our timer period (in ns) to ktime
	ktime = ktime_set(0, g_fusb_timer_tick_period_ns);
	// Start the timer
	hrtimer_start(&chip->timer_state_machine, ktime, HRTIMER_MODE_REL);
}

void fusb_StopTimers(void)
{
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n",
		       __func__);
		return;
	}
	mutex_lock(&chip->lock);
	if (hrtimer_active(&chip->timer_state_machine) != 0) {
		ret = hrtimer_cancel(&chip->timer_state_machine);
		printk(KERN_DEBUG "%s - Active state machine hrtimer canceled: %d\n",
		       __func__, ret);
	}
	if (hrtimer_is_queued(&chip->timer_state_machine) != 0) {
		ret = hrtimer_cancel(&chip->timer_state_machine);
		printk(KERN_DEBUG "%s - Queued state machine hrtimer canceled: %d\n",
		       __func__, ret);
	}
	mutex_unlock(&chip->lock);
	printk(KERN_DEBUG "FUSB  %s - Timer stopped!\n", __func__);
}

// Get the max value that we can delay in 10us increments at compile time
static const unsigned int MAX_DELAY_10US = (UINT_MAX / 10);
void fusb_Delay10us(u32 delay10us)
{
	unsigned int us = 0;
	if (delay10us > MAX_DELAY_10US) {
		printk(KERN_ALERT "%s - Error: Delay of '%u' is too long! Must be less than '%u'.\n",
		       __func__, delay10us, MAX_DELAY_10US);
		return;
	}

	// Convert to microseconds (us)
	us = delay10us * 10;

	// Best practice is to use udelay() for < ~10us times
	if (us <= 10) {
		// BLOCKING delay for < 10us
		udelay(us);
	} else if (us < 20000) {
		// Best practice is to use usleep_range() for 10us-20ms
		// TODO - optimize this range, probably per-platform
		// Non-blocking sleep for at least the requested time, and up to the requested time + 10%
		usleep_range(us, us + (us / 10));
	} else {
		// Best practice is to use msleep() for > 20ms
		// Convert to ms. Non-blocking, low-precision sleep
		msleep(us / 1000);
	}
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/********************************************        SysFS Interface        ******************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/

/*******************************************************************************
* Function:        fusb_Sysfs_Handle_Read
* Input:           output: Buffer to which the output will be written
* Return:          Number of chars written to output
* Description:     Reading this file will output the most recently saved hostcomm output buffer
********************************************************************************/
static ssize_t _fusb_Sysfs_Hostcomm_show(struct device* dev,
					 struct device_attribute* attr,
					 char* buf)
{
	int i = 0;
	int numChars = 0;
	char tempBuf[6] = { 0 };
	struct fusb30x_chip* chip = fusb30x_GetChip();

	if (chip == NULL) {
		printk(KERN_ERR "%s - Chip structure is null!\n", __func__);
	} else if (buf == NULL || chip->HostCommBuf == NULL) {
		printk(KERN_ERR "%s - Buffer is null!\n", __func__);
	} else {
		for (i = 0; i < FSC_HOSTCOMM_BUFFER_SIZE; i++) {
			// Copy 1 byte + null term
			numChars += scnprintf(tempBuf, 6 * sizeof(char),
					      "0x%02x ", chip->HostCommBuf[i]);
			// Append each number to the output buffer
			strcat(buf, tempBuf);
		}
		// Append a newline for pretty++
		strcat(buf, "\n");
		// Account for newline
		numChars++;
	}
	return numChars;
}

/*******************************************************************************
* Function:        fusb_Sysfs_Handle_Write
* Input:           input: Buffer passed in from OS (space-separated list of 8-bit hex values)
*                  size: Number of chars in input
*                  output: Buffer to which the output will be written
* Return:          Number of chars written to output
* Description:     Performs hostcomm duties, and stores output buffer in chip structure
********************************************************************************/
static ssize_t _fusb_Sysfs_Hostcomm_store(struct device* dev,
					  struct device_attribute *attr,
					  const char *input, size_t size)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	char tempByte = 0;
	int numBytes = 0;
	char temp[6] = { 0 };   // Temp buffer to parse out individual hex numbers, +1 for null terminator
	char temp_input[FSC_HOSTCOMM_BUFFER_SIZE] = { 0 };
	char output[FSC_HOSTCOMM_BUFFER_SIZE] = { 0 };
	struct fusb30x_chip* chip = fusb30x_GetChip();

	if (chip == NULL) {
		printk(KERN_ERR "%s - Chip structure is null!\n", __func__);
	} else if (input == NULL) {
		printk(KERN_ERR "%s - Error: Input buffer is NULL!\n", __func__);
	} else {
		// Convert the buffer to hex values
		for (i = 0; i < size; i = i + j) {
			// Parse out a hex number (at most 5 chars: "0x## ")
			for (j = 0; (j < 5) && (j + i < size); j++) {
				// End of the hex number (space-delimited)
				if (input[i + j] == ' ') {
					// We found a space, stop copying this number and convert it
					break;
				}

				temp[j] = input[i + j];     // Copy the non-space byte into the temp buffer
			}

			temp[++j] = 0;                  // Add a null terminator and move past the space

			// We have a hex digit (hopefully), now convert it
			ret = kstrtou8(temp, 16, &tempByte);
			if (ret != 0) {
				printk(KERN_ERR "FUSB  %s - Error: Hostcomm input is not a valid hex value! Return: '%d'\n", __func__, ret);
				return 0;  // Quit on error
			} else {
				temp_input[numBytes++] = tempByte;
				if (numBytes >= FSC_HOSTCOMM_BUFFER_SIZE) {
					break;
				}
			}
		}

		// Handle the message
		fusb_ProcessMsg(temp_input, output);
		// Copy input into temp buffer
		memcpy(chip->HostCommBuf, output, FSC_HOSTCOMM_BUFFER_SIZE);
	}

	return size;
}

// Define our device attributes to export them to sysfs
static DEVICE_ATTR(fusb30x_hostcomm, S_IRWXU | S_IRWXG | S_IROTH,
		   _fusb_Sysfs_Hostcomm_show, _fusb_Sysfs_Hostcomm_store);

void fusb_Sysfs_Init(void)
{
	int ret = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL) {
		printk(KERN_ERR "%s - Chip structure is null!\n", __func__);
		return;
	}
	ret = device_create_file(&chip->client->dev, &dev_attr_fusb30x_hostcomm);
	if (ret != 0) {
		dev_err(&chip->client->dev, "FUSB  %s - Error: Unable to initialize sysfs device file 'fusb30x_io'! ret = %d\n", __func__, ret);
		return;
	}
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/********************************************        Driver Helpers         ******************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void fusb_InitializeCore(void)
{
	core_initialize();
	printk(KERN_DEBUG "FUSB  %s - Core is initialized!\n", __func__);
	fusb_StartTimers();
	printk(KERN_DEBUG "FUSB  %s - Timers are started!\n", __func__);
	core_enable_typec(TRUE);
	printk(KERN_DEBUG "FUSB  %s - Type-C State Machine is started!\n", __func__);
}

bool fusb_IsDeviceValid(void)
{
	unsigned char val = 0;
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return FALSE;
	}

	// Test to see if we can do a successful I2C read
	if (!fusb_I2C_ReadData((unsigned char)0x01, &val))
	{
	printk(KERN_ALERT "FUSB  %s - Error: Could not communicate with device over I2C!\n", __func__);
	return FALSE;
	}

	return TRUE;
}

void fusb_InitChipData(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (chip == NULL) {
		printk(KERN_ALERT "%s - Chip structure is null!\n", __func__);
		return;
	}

#ifdef DEBUG
	chip->dbgTimerTicks = 0;
	chip->dbgTimerRollovers = 0;
	chip->dbgSMTicks = 0;
	chip->dbgSMRollovers = 0;
	chip->dbg_gpio_StateMachine = false;
#endif  // DEBUG

	/* GPIO Defaults */
	chip->gpio_VBus5V_value = false;
	chip->gpio_VBusOther_value = false;

	/* I2C Configuration */
	chip->InitDelayMS = INIT_DELAY_MS;                                              // Time to wait before device init
	chip->numRetriesI2C = RETRIES_I2C;                                              // Number of times to retry I2C reads and writes
}


/*********************************************************************************************************************/
/*********************************************************************************************************************/
/********************************************       Threading Helpers       ******************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/

/*******************************************************************************
* Function:        _fusb_InitWorker
* Input:           delayed_work - passed in from OS
* Return:          none
* Description:     Callback for the init worker, kicks off the main worker
********************************************************************************/
void _fusb_InitWorker(struct work_struct* delayed_work)
{
	struct fusb30x_chip* chip = container_of(delayed_work, struct fusb30x_chip, init_worker.work);
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	// Schedule to kick off the main working thread
	schedule_work(&chip->worker);
}

/*******************************************************************************
* Function:        _fusb_MainWorker
* Input:           delayed_work - passed in from OS
* Return:          none
* Description:     Activates the core
********************************************************************************/
void _fusb_MainWorker(struct work_struct* work)
{
	struct fusb30x_chip* chip = container_of(work, struct fusb30x_chip, worker);
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

#ifdef DEBUG
	// Optionally toggle debug GPIO when SM is called to measure thread tick rate
	dbg_fusb_GPIO_Set_SM_Toggle(!chip->dbg_gpio_StateMachine_value);

	// Tick our state machine tick counter
	if (chip->dbgSMTicks++ >= U8_MAX) {
		// Record a moderate amount of rollovers
		chip->dbgSMRollovers++; 
	}
#endif  // DEBUG

	core_state_machine();                                               // Run the state machine
	schedule_work(&chip->worker);                                       // Reschedule ourselves to run again
}

void fusb_InitializeWorkers(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	printk(KERN_DEBUG "FUSB  %s - Initializing threads!\n", __func__);
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}

	// Initialize our delayed_work and work structs
	INIT_DELAYED_WORK(&chip->init_worker, _fusb_InitWorker);
	INIT_WORK(&chip->worker, _fusb_MainWorker);
}

void fusb_StopThreads(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}
	// Cancel the initial delayed work
	cancel_delayed_work_sync(&chip->init_worker);
	flush_delayed_work(&chip->init_worker);

	// Cancel the main worker
	flush_work(&chip->worker);
	cancel_work_sync(&chip->worker);
}

void fusb_ScheduleWork(void)
{
	struct fusb30x_chip* chip = fusb30x_GetChip();
	if (!chip) {
		printk(KERN_ALERT "FUSB  %s - Error: Chip structure is NULL!\n", __func__);
		return;
	}
	schedule_delayed_work(&chip->init_worker, msecs_to_jiffies(chip->InitDelayMS));
}
