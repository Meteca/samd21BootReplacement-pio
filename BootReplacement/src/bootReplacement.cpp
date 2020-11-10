/* BEWARE: the program will start when you open serial monitor.
 * It will reset the board when needed. Follow the instructions
 * on serial monitor to check the status.
 * 
 * If a reset occurs PIO serial monitor will throw an error.
 * Make sure to check the whole serial monitor output to see
 * messages coming from the board.
 */

#include <Arduino.h>
#include "binaryToWrite.h"

#define NVM_MEMORY			    ((volatile uint16_t *)FLASH_ADDR)
#define BOOTLOADER_SIZE			0x00002000		//8192 byte

#define USER_WORD_0_DEFAULT ((uint32_t)0xd8e0c7ffLU)
#define USER_WORD_1_DEFAULT ((uint32_t)0xfffffc5dLU)
#define USER_WORD_0_BOOT_LOCK ((uint32_t)0xd8e0c7faLU)
#define USER_WORD_1_BOOT_LOCK ((uint32_t)0xfffffc5dLU)

#define Assert(expr) ((void) 0)

#define cpu_irq_is_enabled()    (__get_PRIMASK() == 0)
#define cpu_irq_enable()   \
  do {                     \
    __DMB();               \
    __enable_irq();        \
  } while (0)

#define cpu_irq_disable()  \
  do {                     \
    __disable_irq();       \
    __DMB();               \
  } while (0)

static volatile uint32_t cpu_irq_critical_section_counter;
static volatile bool     cpu_irq_prev_interrupt_state;

typedef enum  status_code {
  STATUS_OK = 0x00,
  STATUS_VALID_DATA = 0x01,
  STATUS_NO_CHANGE = 0x02,
  STATUS_ABORTED = 0x04,
  STATUS_BUSY = 0x05,
  STATUS_SUSPEND = 0x06,
  STATUS_ERR_IO = 0x10,
  STATUS_ERR_INVALID_ARG = 0x17,
  STATUS_ERR_BAD_ADDRESS = 0x18,

} status_code_e;

/**
   NVM controller power reduction mode configurations.
   Power reduction modes of the NVM controller, to conserve power while the device is in sleep.
*/
enum nvm_sleep_power_mode {
  /** NVM controller exits low power mode on first access after sleep. */
  NVM_SLEEP_POWER_MODE_WAKEONACCESS  = NVMCTRL_CTRLB_SLEEPPRM_WAKEONACCESS_Val,
  /** NVM controller exits low power mode when the device exits sleep mode. */
  NVM_SLEEP_POWER_MODE_WAKEUPINSTANT = NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT_Val,
  /** Power reduction mode in the NVM controller disabled. */
  NVM_SLEEP_POWER_MODE_ALWAYS_AWAKE  = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val,
};

/**
 * \brief NVM controller cache readmode configuration.
 *
 * Control how the NVM cache prefetch data from flash.
 *
 */
enum nvm_cache_readmode {
  /** The NVM Controller (cache system) does not insert wait states on
   *  a cache miss. Gives the best system performance.
   */
  NVM_CACHE_READMODE_NO_MISS_PENALTY,
  /** Reduces power consumption of the cache system, but inserts a
   *  wait state each time there is a cache miss.
   */
  NVM_CACHE_READMODE_LOW_POWER,
  /** The cache system ensures that a cache hit or miss takes the same
   *  amount of time, determined by the number of programmed flash
   *  wait states.
   */
  NVM_CACHE_READMODE_DETERMINISTIC,
};

/**
 * \brief NVM controller commands.
 */
enum nvm_command {
	/** Erases the addressed memory row. */
	NVM_COMMAND_ERASE_ROW                  = NVMCTRL_CTRLA_CMD_ER,

	/** Write the contents of the page buffer to the addressed memory page. */
	NVM_COMMAND_WRITE_PAGE                 = NVMCTRL_CTRLA_CMD_WP,

	/** Erases the addressed auxiliary memory row.
	 *
	 *  \note This command can only be given when the security bit is not set.
	 */
	NVM_COMMAND_ERASE_AUX_ROW              = NVMCTRL_CTRLA_CMD_EAR,

	/** Write the contents of the page buffer to the addressed auxiliary memory
	 *  row.
	 *
	 *  \note This command can only be given when the security bit is not set.
	 */
	NVM_COMMAND_WRITE_AUX_ROW              = NVMCTRL_CTRLA_CMD_WAP,

	/** Locks the addressed memory region, preventing further modifications
	 *  until the region is unlocked or the device is erased.
	 */
	NVM_COMMAND_LOCK_REGION                = NVMCTRL_CTRLA_CMD_LR,

	/** Unlocks the addressed memory region, allowing the region contents to be
	 *  modified.
	 */
	NVM_COMMAND_UNLOCK_REGION              = NVMCTRL_CTRLA_CMD_UR,

	/** Clears the page buffer of the NVM controller, resetting the contents to
	 *  all zero values.
	 */
	NVM_COMMAND_PAGE_BUFFER_CLEAR          = NVMCTRL_CTRLA_CMD_PBC,

	/** Sets the device security bit, disallowing the changing of lock bits and
	 *  auxiliary row data until a chip erase has been performed.
	 */
	NVM_COMMAND_SET_SECURITY_BIT           = NVMCTRL_CTRLA_CMD_SSB,

	/** Enter power reduction mode in the NVM controller to reduce the power
	 *  consumption of the system.
	 */
	NVM_COMMAND_ENTER_LOW_POWER_MODE       = NVMCTRL_CTRLA_CMD_SPRM,

	/** Exit power reduction mode in the NVM controller to allow other NVM
	 *  commands to be issued.
	 */
	NVM_COMMAND_EXIT_LOW_POWER_MODE        = NVMCTRL_CTRLA_CMD_CPRM,
#ifdef FEATURE_NVM_RWWEE
	/** Read while write(RWW) EEPROM area erase row. */
	NVM_COMMAND_RWWEE_ERASE_ROW            = NVMCTRL_CTRLA_CMD_RWWEEER,
	/** RWW EEPROM write page. */
	NVM_COMMAND_RWWEE_WRITE_PAGE           = NVMCTRL_CTRLA_CMD_RWWEEWP,
#endif
};


/**
   \internal Internal device instance struct

   This struct contains information about the NVM module which is
   often used by the different functions. The information is loaded
   into the struct in the nvm_init() function.
*/
struct _nvm_module {
  // Number of bytes contained per page.
  uint16_t page_size;
  // Total number of pages in the NVM memory.
  uint16_t number_of_pages;
  // If false, a page write command will be issued automatically when the page buffer is full.
  bool manual_page_write;
};
static struct _nvm_module _nvm_dev;

// Fw status struct
typedef struct {
  uint32_t size;
  uint32_t dl_size;
  uint32_t to_write_sz;
  uint32_t StartAddr;
  uint32_t CurrAddr;
  uint32_t EndAddr;
  uint8_t *pkt_buff_ptr;
} fw_status_s;
fw_status_s firmware;

/*
   Disables global interrupts. To support nested critical sections, an internal
   count of the critical section nesting will be kept, so that global interrupts
   are only re-enabled upon leaving the outermost nested critical section.
*/
static inline void system_interrupt_enter_critical_section (void)
{
  if (cpu_irq_critical_section_counter == 0) {
    if (cpu_irq_is_enabled()) {
      cpu_irq_disable();
      cpu_irq_prev_interrupt_state = true;
    } else {
      /* Make sure the to save the prev state as false */
      cpu_irq_prev_interrupt_state = false;
    }

  }

  cpu_irq_critical_section_counter++;
}

/*
   Enables global interrupts. To support nested critical sections, an internal
   count of the critical section nesting will be kept, so that global interrupts
   are only re-enabled upon leaving the outermost nested critical section.
*/
static inline void system_interrupt_leave_critical_section (void)
{
  // Check if the user is trying to leave a critical section when not in a critical section
  Assert(cpu_irq_critical_section_counter > 0);

  cpu_irq_critical_section_counter--;

  /* Only enable global interrupts when the counter reaches 0 and the state of the global interrupt flag
     was enabled when entering critical state */
  if ((cpu_irq_critical_section_counter == 0) && (cpu_irq_prev_interrupt_state)) {
    cpu_irq_enable();
  }
}

/*

*/
static inline bool nvm_is_ready(void)
{
  return NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY;
}

/*

*/
status_code_e nvm_execute_command(
		const enum nvm_command command,
		const uint32_t address,
		const uint32_t parameter)
{
	uint32_t temp;

	// Check that the address given is valid
	if (address > ((uint32_t)_nvm_dev.page_size * _nvm_dev.number_of_pages)
		&& !(address >= NVMCTRL_AUX0_ADDRESS && address <= NVMCTRL_AUX1_ADDRESS )){
		return STATUS_ERR_BAD_ADDRESS;
	}

	// turn off cache before issuing flash commands
	temp = NVMCTRL->CTRLB.reg;
	NVMCTRL->CTRLB.reg = temp | NVMCTRL_CTRLB_CACHEDIS;

	// Clear error flags
	NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

	// Check if the module is busy
	if (!nvm_is_ready()) {
		return STATUS_BUSY;
	}

	switch (command) {
		// Commands requiring address (protected)
		case NVM_COMMAND_ERASE_AUX_ROW:
		case NVM_COMMAND_WRITE_AUX_ROW:

			// Auxiliary space cannot be accessed if the security bit is set
			if (NVMCTRL->STATUS.reg & NVMCTRL_STATUS_SB) {
				return STATUS_ERR_IO;
			}

			// Set address, command will be issued elsewhere
			NVMCTRL->ADDR.reg = (uintptr_t) & NVM_MEMORY[address / 4];
			break;

		// Commands requiring address (unprotected)
		case NVM_COMMAND_ERASE_ROW:
		case NVM_COMMAND_WRITE_PAGE:
		case NVM_COMMAND_LOCK_REGION:
		case NVM_COMMAND_UNLOCK_REGION:
			// Set address, command will be issued elsewhere
			NVMCTRL->ADDR.reg = (uintptr_t) & NVM_MEMORY[address / 4];
			break;

		// Commands not requiring address
		case NVM_COMMAND_PAGE_BUFFER_CLEAR:
		case NVM_COMMAND_SET_SECURITY_BIT:
		case NVM_COMMAND_ENTER_LOW_POWER_MODE:
		case NVM_COMMAND_EXIT_LOW_POWER_MODE:
			break;

		default:
			return STATUS_ERR_INVALID_ARG;
	}

	// Set command
	NVMCTRL->CTRLA.reg = command | NVMCTRL_CTRLA_CMDEX_KEY;

	// Wait for the nvm controller to become ready
	while (!nvm_is_ready()) {
	}

	// restore the setting
	NVMCTRL->CTRLB.reg = temp;

	return STATUS_OK;
}

/*

*/
status_code_e configure_nvm (void)
{
  status_code_e error_code = STATUS_OK;
  // Turn on the digital interface clock
  PM->APBBMASK.reg |= PM_APBBMASK_NVMCTRL;

  // Clear error flags
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

  // Check if the module is busy
  if (!nvm_is_ready()) {
    error_code = STATUS_BUSY;
  }

  // Writing configuration to the CTRLB register
  NVMCTRL->CTRLB.reg =
    NVMCTRL_CTRLB_SLEEPPRM(NVM_SLEEP_POWER_MODE_WAKEONACCESS)   |
    ((true & 0x01) << NVMCTRL_CTRLB_MANW_Pos)                   |
    NVMCTRL_CTRLB_RWS(NVMCTRL->CTRLB.bit.RWS)                   |
    ((false & 0x01) << NVMCTRL_CTRLB_CACHEDIS_Pos)              |
    NVMCTRL_CTRLB_READMODE(NVM_CACHE_READMODE_NO_MISS_PENALTY);

  // Initialize the nvm struct
  _nvm_dev.page_size         = (8 << NVMCTRL->PARAM.bit.PSZ);
  _nvm_dev.number_of_pages   = NVMCTRL->PARAM.bit.NVMP;
  _nvm_dev.manual_page_write = true;

  // If the security bit is set, the auxiliary space cannot be written
  if (NVMCTRL->STATUS.reg & NVMCTRL_STATUS_SB) {
    error_code = STATUS_ERR_IO;
  }

  // Turn on the digital interface clock
  PM->APBBMASK.reg |= PM_APBBMASK_NVMCTRL;

  // Clear error flags
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

  // Check if the module is busy
  if (!nvm_is_ready()) {
    error_code = STATUS_BUSY;
  }

  // Writing configuration to the CTRLB register
  NVMCTRL->CTRLB.reg =
    NVMCTRL_CTRLB_SLEEPPRM(NVM_SLEEP_POWER_MODE_WAKEONACCESS)	|
    ((false & 0x01) << NVMCTRL_CTRLB_MANW_Pos)					|
    NVMCTRL_CTRLB_RWS(NVMCTRL->CTRLB.bit.RWS)					|
    ((false & 0x01) << NVMCTRL_CTRLB_CACHEDIS_Pos)				|
    NVMCTRL_CTRLB_READMODE(NVM_CACHE_READMODE_NO_MISS_PENALTY);

  return error_code;
}

/*

*/
static status_code_e partition_erase (uint32_t start_addr, uint32_t end_addr)
{
  status_code_e error_code = STATUS_OK;

  for (uint32_t flash_addr = start_addr; flash_addr < end_addr; flash_addr += NVMCTRL_ROW_SIZE) {
    do {
      error_code = (status_code_e)nvm_is_ready();
      NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

      // Set address and command
      NVMCTRL->ADDR.reg  = (uintptr_t)&NVM_MEMORY[flash_addr >> 2];
      NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX_KEY;
    } while (error_code == STATUS_BUSY);
  }
  return error_code;
}

/**
   Writes a number of bytes from a buffer to a page in the NVM memory region.

   \param[in]  destination_address  Destination page address to write to
   \param[in]  buffer               Pointer to buffer where the data to write is stored
   \param[in]  length               Number of bytes in the page to write

   \note If writing to a page that has previously been written to, the page's
         row should be erased (via \ref nvm_erase_row()) before attempting to
         write new data to the page.

   \note For SAMD21 RWW devices, see \c SAMD21_64K, command \c NVM_COMMAND_RWWEE_WRITE_PAGE
   must be executed before any other commands after writing a page,
   refer to errata 13588.

   \note If manual write mode is enable, write command must be executed after
   this function, otherwise the data will not write to NVM from page buffer.

   \return Status of the attempt to write a page.
   \retval STATUS_OK               Requested NVM memory page was successfully read
   \retval STATUS_BUSY             NVM controller was busy when the operation was attempted
   \retval STATUS_ERR_BAD_ADDRESS  The requested address was outside the
                                   acceptable range of the NVM memory region or
                                   not aligned to the start of a page
   \retval STATUS_ERR_INVALID_ARG  The supplied write length was invalid
*/
status_code_e nvm_write_buffer(const uint32_t destination_address, const uint8_t *buffer, uint16_t length)
{
  // Check if the destination address is valid
  if (destination_address > ((uint32_t)_nvm_dev.page_size * _nvm_dev.number_of_pages)) {
    return STATUS_ERR_BAD_ADDRESS;
  }

  // Check if the write address not aligned to the start of a page
  if (destination_address & (_nvm_dev.page_size - 1)) {
    return STATUS_ERR_BAD_ADDRESS;
  }

  // Check if the write length is longer than a NVM page
  if (length > _nvm_dev.page_size) {
    return STATUS_ERR_INVALID_ARG;
  }

  // Check if the module is busy
  if (!nvm_is_ready()) {
    return STATUS_BUSY;
  }

  // Erase the page buffer before buffering new data
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX_KEY;

  // Check if the module is busy
  while (!nvm_is_ready()) {
    // Force-wait for the buffer clear to complete
  }

  // Clear error flags
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;

  uint32_t nvm_address = (destination_address >> 1);

  // NVM _must_ be accessed as a series of 16-bit words, perform manual copy to ensure alignment
  for (uint16_t i = 0; i < length; i += 2) {
    uint16_t data;

    // Copy first byte of the 16-bit chunk to the temporary buffer
    data = buffer[i];

    // If we are not at the end of a write request with an odd byte count, store the next byte of data as well
    if (i < (length - 1)) {
      data |= (buffer[i + 1] << 8);
    }

    // Store next 16-bit chunk to the NVM memory space
    NVM_MEMORY[nvm_address++] = data;
  }

  /* If automatic page write mode is enable, then perform a manual NVM
     write when the length of data to be programmed is less than page size */
  if ((_nvm_dev.manual_page_write == false) && (length < NVMCTRL_PAGE_SIZE)) {
    return nvm_execute_command(NVM_COMMAND_WRITE_PAGE, destination_address, 0);   // issue command write page
  }

  return STATUS_OK;
}

/*

*/
status_code_e flash_write (uint16_t bytes_to_write, uint32_t flash_addr, const uint8_t * buffer)
{
  system_interrupt_enter_critical_section();
  uint8_t n_pages = bytes_to_write >> 6;
  uint8_t rest = bytes_to_write % FLASH_PAGE_SIZE;
  if (rest != 0)
    n_pages += 1;

  for (uint8_t i = 0; i < n_pages; i++) {
    nvm_write_buffer(flash_addr, buffer + (i * FLASH_PAGE_SIZE), FLASH_PAGE_SIZE);
    flash_addr += FLASH_PAGE_SIZE;
  }
  system_interrupt_leave_critical_section();

  return STATUS_OK;
}

/*

*/
static void fuseSet(uint32_t user_word_0, uint32_t user_word_1)
{
  uint32_t temp = 0;  // Auxiliary space cannot be accessed if the security bit is set
  if (NVMCTRL->STATUS.reg & NVMCTRL_STATUS_SB) {
    return;
  }

  // Disable Cache
  temp = NVMCTRL->CTRLB.reg;  NVMCTRL->CTRLB.reg = temp | NVMCTRL_CTRLB_CACHEDIS;  // Clear error flags
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;  // Set address, command will be issued elsewhere
  NVMCTRL->ADDR.reg = (NVMCTRL_AUX0_ADDRESS >> 1);  // Erase the user page
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_EAR | NVMCTRL_CTRLA_CMDEX_KEY;  // Wait for NVM command to complete
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));  // Clear error flags
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;  // Set address, command will be issued elsewhere
  NVMCTRL->ADDR.reg = (NVMCTRL_AUX0_ADDRESS >> 1);  // Erase the page buffer before buffering new data
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_PBC | NVMCTRL_CTRLA_CMDEX_KEY;  // Wait for NVM command to complete
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));  // Clear error flags
  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;  // Set address, command will be issued elsewhere
  NVMCTRL->ADDR.reg = (NVMCTRL_AUX0_ADDRESS >> 1); 
  // write the default fuses values to the memory buffer
  *((uint32_t *)NVMCTRL_AUX0_ADDRESS) = user_word_0;
  *(((uint32_t *)NVMCTRL_AUX0_ADDRESS) + 1) = user_word_1;  // Write the user page
  NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMD_WAP | NVMCTRL_CTRLA_CMDEX_KEY;  // Restore the settings
  NVMCTRL->CTRLB.reg = temp;
}

void setup (void)
{
  Serial.begin(115200);
  while(!Serial);
  delay(500);
  Serial.print("flash config...");
  configure_nvm();
  Serial.println("done!");

  firmware.StartAddr = FLASH_ADDR;
  firmware.EndAddr = BOOTLOADER_SIZE;
  firmware.CurrAddr = firmware.StartAddr;
  firmware.size = sizeof(binary2Write) / sizeof(binary2Write[0]);
  firmware.dl_size = firmware.to_write_sz = 0;

  // check if bootprot fuse is disabled
  if ((*((uint32_t *)NVMCTRL_AUX0_ADDRESS) != USER_WORD_0_DEFAULT) ||
      (*(((uint32_t *)NVMCTRL_AUX0_ADDRESS) + 1) != USER_WORD_1_DEFAULT)) {
        Serial.print("set fuses to default...");
        fuseSet(USER_WORD_0_DEFAULT, USER_WORD_1_DEFAULT);
        Serial.println("done!");
        Serial.println("Fuses set to default state. The board is going to reset to make changes effective.");
        Serial.println();
        Serial.println("Please close and open again the serial terminal to continue the uptading.");
        delay(100);
        NVIC_SystemReset();
  }
  Serial.print("partition erase...");
  partition_erase(firmware.StartAddr, firmware.EndAddr);
  Serial.println("done!");
  Serial.print("writing flash...");
  if (flash_write(firmware.size, firmware.CurrAddr, binary2Write) == STATUS_OK){
      Serial.println("done!");
      Serial.print("restoring fuses...");
      fuseSet(USER_WORD_0_BOOT_LOCK, USER_WORD_1_BOOT_LOCK);
      Serial.println("done!");
  }
  Serial.print("Bootloader has been updated. The board is now going to reset. Load a new sketch in the board to try new bootloader features.");
  delay(200);
  NVIC_SystemReset();
}

void loop (void)
{
}
