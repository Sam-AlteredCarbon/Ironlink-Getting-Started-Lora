/**
 * @file ironlink-library.h
 * @author Sam Onwugbenu (support@ironlink.io)
 * @brief Ironlink Library
 * @version 1.1
 * @date 2021-05-16
 * 
 * @copyright Copyright (c) 2021 Altitude Tech - http://www.altitude.tech
 *   All rights reserved.
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions are met:
 *	1. Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	2. Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	3. All advertising materials mentioning features or use of this software
 *	   must display the following acknowledgement:
 *	   This product includes software developed by the Altitude Tech.
 *	4. Neither the name of the Altitude Tech nor the
 *	   names of its contributors may be used to endorse or promote products
 *	   derived from this software without specific prior written permission.
 *	THIS SOFTWARE IS PROVIDED BY ALTITUDE TECH ''AS IS'' AND ANY
 *	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL ALTITUDE TECH BE LIABLE FOR ANY
 *	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef INC_IRONLINK_IRONLINK_LIBRARY_H_
#define INC_IRONLINK_IRONLINK_LIBRARY_H_

// Includes
#include "main.h"

// Peripherials Libraries
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "dma.h"

// USB comms Libraries
#include "usbd_cdc_if.h"
#include "usb_device.h"

// FreeRTOS Libraries
#include "cmsis_os.h"

// Printf Libraries
#include <string.h>
#include <stdarg.h>

/************************************* Defines *******************************/

// Bootloader Defines
/**
 * @defgroup  Ironlink_General
 * @brief Ironlink Global Variables, Pointers and Functions
 * @{ 
 */
#define BOOTLOADER					//!<Macro to tell the complier to configure the system to be used with the usb bootloader

// System Defines
#define FREERTOS					//!<Macro to using freertos specific functions instead of hal, used when threading is enabled


#define MAX_BUFFER 					64 				//!<General max buffer size
/**@} */
//GPIO Defines
/**
 * @defgroup  Ironlink_GPIO
 * @brief Ironlink GPIO Related Variables, Pointers and Functions
 * @{ 
 */
/**
 * @defgroup  Ironlink_GPIO_Defines
 * @brief Ironlink UART Related Variables, Pointers and Functions
 * @{
 */
#define GPIO1						GPIO_PIN_4		//!<Mapping GPIO1 pin name to the HAL definition
#define GPIO1_PORT					GPIOA			//!<Mapping GPIO1 port name to the HAL definition

#define GPIO2						GPIO_PIN_8		//!<Mapping GPIO2 pin name to the HAL definition
#define GPIO2_PORT					GPIOA			//!<Mapping GPIO2 port name to the HAL definition

#define GPIO3						GPIO_PIN_0		//!<Mapping GPIO3 pin name to the HAL definition
#define GPIO3_PORT					GPIOB			//!<Mapping GPIO3 port name to the HAL definition

#define GPIO4						GPIO_PIN_2		//!<Mapping GPIO4 pin name to the HAL definition
#define GPIO4_PORT					GPIOB			//!<Mapping GPIO4 port name to the HAL definition

#define GPIO5						GPIO_PIN_3		//!<Mapping GPIO5 pin name to the HAL definition
#define GPIO5_PORT					GPIOB			//!<Mapping GPIO5 port name to the HAL definition

#define GPIO7						GPIO_PIN_15		//!<Mapping GPIO7 pin name to the HAL definition
#define GPIO7_PORT					GPIOB			//!<Mapping GPIO7 port name to the HAL definition
/**@} */
/**@} */

/**
 * @defgroup  Ironlink_ADC
 * @brief Ironlink ADC Related Variables, Pointers and Functions
 * @{ 
 */
/**
 * @defgroup  Ironlink_ADC_Defines
 * @brief Ironlink UART Related Variables, Pointers and Functions
 * @{
 */
#define 	IL_ADC_AUTO_START						//!<Macro to start the adc at boot, comment out if you don't want the adc starting at boot
#define		IL_ADC_DMA								//!<Macro to start the adc in DMA mode, comment out if using ADC in polling mode

#define 	NUMCHANNELS			2					//!<Number of adc channel being used Max:3
//TODO:Change this to enum
#define		IL_ADC_CHANNEL_1	0					//!<Definition for adc channel 1
#define		IL_ADC_CHANNEL_2	1					//!<Definition for adc channel 2
#define		IL_ADC_CHANNEL_3	2					//!<Definition for adc channel 3

// PA4 - GPIO 1 - ADC 1
// PA1 - UART4 RX - ADC 2
// PB0 - GPIO 3 - ADC 3
/**@} */
/**@} */

/**
 * @defgroup  Ironlink_UART
 * @brief Ironlink UART Related Variables, Pointers and Functions
 * @{ 
 */

/**
 * @defgroup  Ironlink_UART_Defines
 * @brief Ironlink UART Related Variables, Pointers and Functions
 * @{ 
 */
#define IL_UART_INTERRUPT_MODE						//!<Macro to use UART the peripherial in interrupt mode.
#define IL_UART_OBJECTS_PREDEFINED					//!<Macro to predefine the uart objects in library, comment out if you want to define the objects in the main file

#define	UART1						USART1			//!<Map UART1 to HAL USART definition
#define	UART2						USART4			//!<Map UART2 to HAL USART definition
#define	UART_MODEM					USART3			//!<Map UART_Modem to HAL USART definition
#define	UART_GPS					USART2			//!<Map UART_GPS to HAL USART definition
//TODO:Change to MAX_UART_TIMEOUT
#define MAX_UART_TIMEOUT 				10000			//!<Define max UART rx or tx wait time in milliseconds
/**@} */

/**@} */

/**
 * @defgroup  Ironlink_I2C
 * @brief Ironlink I2C Related Variables, Pointers and Functions
 * @{ 
 */

/**
 * @defgroup  Ironlink_I2C_Defines
 * @{ 
 */
#define	IL_I2C_INTERRUPT_MODE						//!<Macro to use the I2C peripherial in interrupt mode 
#define	IL_I2C_OBJECTS_PREDEFINED					//!<Macro to predefine the i2c objects in library, comment out if you want to define the object in the main file             

#define IL_I2C_TIMEOUT				100				//!<Define I2C timeout in milliseconds
#define	IL_I2C_MAX_BUFFER_SIZE		50				//!<Define max I2C buffer size
/**@} */

/**@} */

/**
 * @defgroup  Ironlink_USB
 * @brief Ironlink USB Related Variables, Pointers and Functions
 * @{ 
 */
/**
 * @defgroup  Ironlink_USB_Defines
 * @{ 
 */
#define IL_USB_OBJECTS_PREDEFINED					//!<Macro to predefine the USB objects in library, comment out if you want to define the object in the main file 
/**@} */
/**@} */

/**
 * @defgroup  Ironlink_LORA
 * @brief Ironlink LORA Related Variables, Pointers and Functions
 * @{ 
 */

/**
 * @defgroup  Ironlink_LORA_Defines
 * @{ 
 */

#define	IL_LORA_MODEM_RST_PIN			GPIO_PIN_12 //!<Mapping Lora modem reset pin to the HAL definiton 
#define IL_LORA_MODEM_RST_PORT			GPIOB		//!<Mapping Lora modem reset port to the HAL definiton 

#define IL_LORA_MAX_TRANSMIT_STRING 	30			//!<Max lora transmit string length
#define IL_LORA_MAX_RECV_BUFF 			30			//!<Max lora recieve string length


#define RN_JOIN_OTAA_MODE           "mac join otaa"			//!<Join the network in otaa mode
#define RN_JOIN_ABP_MODE            "mac join abp"          //!<Join the network in abp mode

// LORA SYS Commands
#define RN_SYS_SLEEP                "sys sleep"				//!<Sleep in ms. From 100 to 4294967296
#define RN_SYS_GET_VER              "sys get ver"			//!<Returns the information on hardware platform, firmware version, release date.
#define RN_SYS_FACTORY_RESET        "sys factoryRESET"		//!<Reset all configurations to factory default
#define RN_SYS_RESET                "sys reset"				//!<Reboot module

#define RN_SYS_GET_VDD              "sys get vdd"			//!<Get the mV of the module. From 0 - 3600
#define RN_SYS_GET_HWEUI            "sys get hweui"			//!<Get the HW eui address of the module
#define RN_SYS_SET_PIN              "sys set pindig"		//!<Set PIN. GPIO0 - GPIO14, UART_CTS, UART_RTS, TEST0, TEST1. to 0 or 1.

// LORA MAC Commands
#define RN_MAC_GET_APP_EUI          "mac get appeui"		//!<Get current programmed APP eui.
#define RN_MAC_GET_STATUS           "mac get status"		//!<get the current status of the modem.
#define RN_MAC_RESET_CMD            "mac reset 868"			//!<Reset all configurations for band 868.
#define RN_MAC_TX_CMD               "mac tx"				//!<Send data to Lora network. cnf (confirmed) or uncnf (unconfirmed), port number from 1 to 223, data
#define RN_MAC_TX_UNCONFIRMED		"uncnf"					//!<Send unconfirmed string
#define RN_MAC_TX_CONFIRMED			"cnf"					//!<Send confirmed string
#define RN_MAC_JOIN_CMD             "mac join"				//!<Join Lora network. OTAA (over-the-air activation) or ABP (activation by personalization)
#define RN_MAC_SAVE_CMD             "mac save"				//!<Save the configuration (band, deveui, appeui, appkey, nwkskey, appskey, devaddr, ch).
#define RN_MAC_PAUSE_CMD            "mac pause"				//!<Pause LoraWan stack in ms (must be done if you want to change a config while you are already connected to Lora network). From 0 to 4294967295
#define RN_MAC_RESUME_CMD           "mac resume"			//!<Resume LoraWan stack

#define RN_MAC_SET_DEV_ADDR         "mac set devaddr"		//!<Set device address. from 00000000 to FFFFFFFF
#define RN_MAC_SET_DEV_EUI          "mac set deveui"		//!<Set device identifier. 8-byte HEX
#define RN_MAC_SET_APP_EUI          "mac set appeui"		//!<Set application identifier. value: 8-byte HEX
#define RN_MAC_SET_NWK_SESS_KEY     "mac set nwkskey"		//!<Set the network session key. value: 16-byte HEX
#define RN_MAC_SET_APP_SESS_KEY     "mac set appskey"		//!<Set the application session key. value: 16-byte HEX
#define RN_MAC_SET_APP_KEY          "mac set appkey"		//!<Set the application key. value: 16-byte HEX
#define RN_MAC_SET_DATARATE         "mac set dr"			//!<Set the data rate. value: 0 to 7
#define RN_MAC_SET_ADR_ON_CMD       "mac set adr"			//!<Set the adaptive data rate. value: "on" or "off"
#define RN_MAC_SET_AR_ON_CMD        "mac set ar"			//!<Set automatic reply. value: "on" or "off"
#define RN_MAC_SET_ON 		       	"on"					//!<Send on string
#define RN_MAC_SET_OFF        		"off"					//!<Send off string
#define RN_MAC_SET_PWRIDX			"mac set pwridx"		//!<Set the output power. value: 0 to 5 for 433 MHz and 1 to 5 868 MHz

// LORA Radio commands
#define RN_RADIO_GET_MODE           "radio get mod"			//!<Get the modulation mode
#define RN_RADIO_SET_MODE           "radio set mod"			//!<Set the modulation mode
#define RN_RADIO_MODE_LEN           18						//!<Hard code radio mode lenght
#define RN_RADIO_SET_PWR            "radio set pwr"			//!<Set the output power: value: string, can be sf7, sf8, sf8, sf9, sf10, sf11, sf12
#define RN_RADIO_GET_PWR            "radio get pwr"			//!<Get the output power
#define RN_RADIO_SET_SYNC           "radio set sync"		//!<Set the sync word. value: hexidecimal value lora mode 1 byte fsk mode up to 8 bytes
#define RN_LORA_MODE                "lora"					//!<LoRa mode
#define RN_FSK_MODE                 "fsk"					//!<Frequency key shifting mode

/**@} */
/**@} */


/********************************** Type Defines ****************************/

//DMA Type Defines
/**
 * @defgroup  Ironlink_DMA
 * @{ 
 */
typedef void (*il_dma_init_typedef)();							//!<Create ironlink alias for HAL DMA init function
/**@} */

/**
 * @ingroup  Ironlink_General
 * @brief 
 * @{ 
 */

/**
 * @defgroup  Ironlink_General_Type_Def
 * @brief 
 * @{ 
 */

// System clock Type Defines
typedef uint32_t (*il_get_tick_typedef)();						//!<Create ironlink alias for the  HAL get tick funtion


// Delays Type Defines
typedef osStatus il_delay_status;								//!<Create ironlink alias for freertos osStatus 
typedef il_delay_status (*il_os_delay_typedef)(uint32_t);		//!<Create ironlink alias for the freertos osDelay function 
typedef void (*il_delay_typedef)(uint32_t);						//!<Create ironlink alias for the HAL_Delay function

typedef HAL_StatusTypeDef il_hal_status_typedef;				//!<Create ironlink alias for the HAL status type define 
/**@} */

/**
 * @brief Status options for ironlink functions
 * 
 */

typedef enum il_status_typedef
{
	IL_OK       = 0x00U,		//!<Opertation completed sucessfully
	IL_ERROR    = 0x01U,		//!<Operation exited with an error
	IL_BUSY     = 0x02U,		//!<Operation is still running 
	IL_TIMEOUT  = 0x03U			//!<Operation timed out
} il_status_typedef;
/**@} */

/**
 * @ingroup  Ironlink_GPIO
 * @{ 
 */

/**
 * @defgroup  Ironlink_GPIO_Type_Def
 * @brief 
 * @{ 
 */
typedef GPIO_TypeDef il_gpio_typedef;														//!<Create ironlink alias for HAL GPIO type define
typedef GPIO_PinState il_gpio_pinstate;														//!<Create ironlink alias for HAL GPIO pin state
/**@} */

/**
 * @defgroup  Ironlink_GPIO_Function_Pointer
 * @brief 
 * @{ 
 */
typedef void (*il_gpio_init_typedef)();														//!<Create ironlink alias for the HAL GPIO init function
typedef il_gpio_pinstate (*il_hal_gpio_read_pin_typedef)(il_gpio_typedef*, uint16_t); 		//!<Create ironlink alias for the HAL GPIO pin read function
typedef void(*il_hal_gpio_write_pin_typedef)(il_gpio_typedef*, uint16_t, il_gpio_pinstate);	//!<Create ironlink alias for the HAL GPIO pin write function
/**@} */

/**@} */


/**
 * @defgroup  Ironlink_Unions
 * @brief List of unions used in library
 * @{ 
 */
/**
 * @brief Ironlink union for a byte
 * 
 */
typedef union il_union_byte{
	uint8_t u8_byte;			//!<Unsigned 8 bit / 1 byte integer 
	int8_t i8_byte;				//!<Signed 8 bit / 1 byte integer 
	char ch_byte;				//!<Character
}il_union_byte;
/**
 * @brief Ironlink union for 2 bytes
 * 
 */
typedef union il_union_2byte{
	uint16_t u16_byte;			//!<Unsigned 16 bit / 2 bytes integer 
	int16_t i16_byte;			//!<Signed 16 bit / bytes integer
	char ch_2byte[2];			//!<Character array 2 bytes
	uint8_t u8_2byte[2];		//!<Unsigned array 2 bytes
}il_union_2byte;
/**
 * @brief Ironlink union for 4 bytes
 * 
 */
typedef union il_union_4byte{
	uint32_t u32_byte;			//!<Unsigned 32 bit / 4 bytes integer
	int32_t i32_byte;			//!<Signed 32bit / 4 bytes integer
	float f_byte;				//!<Float
	char ch_4byte[4];			//!<Character array 4 bytes
	uint8_t u8_4byte[4];		//!<Unsigned array 4 bytes
}il_union_4byte;
/**
 * @brief Ironlink union for a buffer of MAX_BUFFER size(Defined in ironlink.h under system defines)
 * 
 */
typedef union il_union_buffer{
	uint8_t u8_buffer[MAX_BUFFER]; 	//!<Unsigned 8 bit / 1 byte array of size MAX_BUFFER
	int8_t i8_buffer[MAX_BUFFER];	//!<Signed 8 bit / 1 byte array of size MAX_BUFFER
	char ch_buffer[MAX_BUFFER];		//!<Character 8 bit / 1 byte array of size MAX_BUFFER
}il_union_buffer;

/**
 * @brief Struct for transmit and recieve buffers impleneted for communcation
 * 
 */
typedef struct il_buffer_typedef{

	// Receive Variables
	uint16_t rx_index;					//!<Current recieve buffer array postion 
	il_union_byte  rx_char;				//!<Recieve charater 
	il_union_buffer  rx_buffer;			//!<Recieve buffer

	// Transmit Variables
	uint16_t tx_length;					//!<Length of transmit string
	il_union_buffer  tx_buffer;			//!<Transmit buffer

	uint32_t timeout;					//!<How long to wait before setting the timeout flag 

	// Flags
	volatile uint8_t  eot_flag;			//!<End of transmission flag
	volatile uint8_t  err_flag;			//!<Error flag 
	volatile uint8_t  irq_flag;			//!<Interrupt routine flag, Not currently used
	volatile uint8_t  timeout_flag;		//!<Timeout reached flag, Not currently used

	//Start and end of line transmission character
	il_union_byte sol_char;
	il_union_byte eol_char;

	// Message length tracking variables
	volatile uint16_t		head;		//!<Keeps track of characters to be read by the uart read function
	volatile uint16_t		tail;		//!<Keeps track of the characters read by the uart read interrupt function
}il_buffer_typedef;

/**@} */

/**
 * @ingroup  Ironlink_ADC
 * @{ 
 */

/**
 * @defgroup Ironlink ADC Type Defines
 * @{
 */

typedef ADC_HandleTypeDef il_adc_handle_typedef;	//!<Create ironlink alias for HAL ADC handle type define 
/**@} */

/**
 * @defgroup  Ironlink_ADC_Function_Pointers
 * @{ 
 */

typedef void(*il_adc_init_typedef)();				//!<Create ironlink alias for HAL ADC init function

typedef il_hal_status_typedef(*il_adc_start_typedef)(il_adc_handle_typedef*); 																//!<Create ironlink alias for HAL ADC start function
typedef il_hal_status_typedef(*il_adc_stop_typedef)(il_adc_handle_typedef*);																//!<Create ironlink alias for HAL ADC stop function 	
typedef il_hal_status_typedef(*il_adc_calibration_typedef)(il_adc_handle_typedef*);															//!<Create ironlink alias for HAL ADC calibration function
typedef uint32_t(*il_adc_get_value_typedef)(il_adc_handle_typedef*);																		//!<Create ironlink alias for HAL ADC get value funtion
typedef il_hal_status_typedef(*il_adc_ready_typedef)(il_adc_handle_typedef*,uint32_t);														//!<Create ironlink alias for HAL ADC ready function
typedef il_hal_status_typedef(*il_adc_start_dma_typedef)(il_adc_handle_typedef*,uint32_t*,uint32_t); 										//!<Create ironlink alias for HAL ADC start in DMA mode function

/**@} */

/**@} */

/**
 * @ingroup  Ironlink_I2C
 * @{ 
 */

/**
 * @defgroup Ironlink_I2C_Type_Def
 * @brief Ironlink I2C Type defines 
 * Used to map HAL type defines to Ironlink type defines
 * @{
 */

typedef I2C_HandleTypeDef il_i2c_handle_typedef;																							//!<Create ironlink alias for HAL I2C handle function
typedef HAL_I2C_StateTypeDef il_i2c_state_typedef;																							//!<Create ironlink alias for HAL ADC init function
/**@} */

/**
 * @defgroup  Ironlink_I2C_Function_Pointers
 * @brief Ironlink I2C Function pointers 
 * Created to map the HAL I2C library functions
 * to functions in the Ironlink Library 
 *
 * @{ 
 */
typedef	void(*il_i2c_init_typedef)();																										//!<Create ironlink alias for HAL I2C init function

typedef	il_hal_status_typedef (*il_i2c_ready_typedef)(il_i2c_handle_typedef*,uint16_t,uint32_t,uint32_t);									//!<Create ironlink alias for HAL I2C ready function
typedef	il_i2c_state_typedef (*il_i2c_get_state_typedef)(il_i2c_handle_typedef*);															//!<Create ironlink alias for HAL I2C get state function
typedef	uint32_t (*il_i2c_get_error_typedef)(il_i2c_handle_typedef*);																		//!<Create ironlink alias for HAL I2C get error function

typedef	il_hal_status_typedef (*il_i2c_write_typedef)(il_i2c_handle_typedef*,uint16_t,uint8_t*,uint16_t,uint32_t);							//!<Create ironlink alias for HAL I2C write function
typedef	il_hal_status_typedef (*il_i2c_write_it_typedef)(il_i2c_handle_typedef*,uint16_t,uint8_t*,uint16_t);								//!<Create ironlink alias for HAL I2C write interrupt function
typedef	il_hal_status_typedef (*il_i2c_write_reg_typedef)(il_i2c_handle_typedef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t); 	//!<Create ironlink alias for HAL I2C write registers function
typedef	il_hal_status_typedef (*il_i2c_write_reg_it_typedef)(il_i2c_handle_typedef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t); 			//!<Create ironlink alias for HAL I2C write registers interrupt mode function

typedef	il_hal_status_typedef (*il_i2c_read_typedef)(il_i2c_handle_typedef*,uint16_t,uint8_t*,uint16_t,uint32_t);							//!<Create ironlink alias for HAL I2C read function
typedef	il_hal_status_typedef (*il_i2c_read_it_typedef)(il_i2c_handle_typedef*,uint16_t,uint8_t*,uint16_t);									//!<Create ironlink alias for HAL I2C read interrupt function
typedef	il_hal_status_typedef (*il_i2c_read_reg_typedef)(il_i2c_handle_typedef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t,uint32_t);		//!<Create ironlink alias for HAL I2C read registers function
typedef	il_hal_status_typedef (*il_i2c_read_reg_it_typedef)(il_i2c_handle_typedef*,uint16_t,uint16_t,uint16_t,uint8_t*,uint16_t);			//!<Create ironlink alias for HAL I2C read registers interrupt mode function
/**@} */

/**
 * @defgroup  Ironlink_I2C_Objects
 * @{ 
 */

/**
 * @brief Ironlink I2C struct
 * Buffers and varaibles used to track the status and
 * send data between the I2C peripherial and the system memory
 */

//TODO:Use general buffer for this instead of custom one
typedef struct {
	uint8_t  tx_buffer[IL_I2C_MAX_BUFFER_SIZE];			//!<Transmit buffer, Buffer size: IL_I2C_MAX_BUFFER_SIZE
	uint8_t  rx_buffer[IL_I2C_MAX_BUFFER_SIZE];			//!<Receive buffer, Buffer size: IL_I2C_MAX_BUFFER_SIZE
	uint16_t tx_length;									//!<Transmit length
	uint16_t rx_length;									//!<Receive length
	uint32_t timeout;									//!<Timeout length in millisecs
}il_i2c_buffer;

/**
 * @brief Ironlink I2C object 
 * 
 */

typedef struct{
	il_i2c_handle_typedef*  i2c_channel;				//!<HAL I2C object
	uint8_t device_address;								//!<I2C address for the device
	uint8_t register_size;								//!<I2C register size 1 for 8 bit and 2 for 16 bit
	uint8_t register_address[2];						//!<I2C address for register on device
	il_i2c_buffer buffer;								//!<Buffer passed to I2C peripherial for transmit or recieve functions
}il_i2c_object;

/**@} */

/**@} */


/**
 * @ingroup  Ironlink_UART
 * @{ 
 */


/**
 * @defgroup  Ironlink_UART_Type_Defs
 * @{ 
 */

typedef UART_HandleTypeDef il_uart_handle_typedef;		//!<Create Ironlink alias for UART handle type define
typedef HAL_UART_StateTypeDef il_uart_state_typedef;	//!<Create Ironlink alias for UART state type define

/**@} */

/**
 * @defgroup  Ironlink_UART_Function_Pointers
 * @brief Ironlink UART Function pointers 
 * Created to map the HAL UART library functions
 * to functions in the Ironlink Library 
 * @{ 
 */
typedef void (*il_uart_init_typedef)();																			//!<Create Ironlink alias for HAL UART init function

typedef il_uart_state_typedef (*il_uart_get_state_typedef)(il_uart_handle_typedef*);							//!<Create Ironlink alias for HAL UART get state function
typedef uint32_t (*il_uart_get_error_typedef)(il_uart_handle_typedef*);											//!<Create Ironlink alias for HAL UART get error function
typedef il_hal_status_typedef (*il_uart_abort_typedef)(il_uart_handle_typedef*);								//!<Create Ironlink alias for HAL UART abort function
//	Polling mode
typedef il_hal_status_typedef (*il_uart_write_typedef)(il_uart_handle_typedef*,uint8_t*,uint16_t,uint32_t);		//!<Create Ironlink alias for HAL UART write function
typedef il_hal_status_typedef (*il_uart_read_typedef)(il_uart_handle_typedef*,uint8_t*,uint16_t,uint32_t);		//!<Create Ironlink alias for HAL UART read function
//	Interrupt mode
typedef il_hal_status_typedef (*il_uart_write_it_typedef)(il_uart_handle_typedef*,uint8_t*,uint16_t);			//!<Create Ironlink alias for HAL UART write interrupt mode function
typedef il_hal_status_typedef (*il_uart_read_it_typedef)(il_uart_handle_typedef*,uint8_t*,uint16_t);			//!<Create Ironlink alias for HAL UART read interrupt mode function
/**@} */

/**
 * @defgroup  Ironlink_UART_Objects
 * @brief Ironlink UART struct
 * Buffers and varaibles used to track the status and
 * send data between the UART peripherial and the system memory
 * @{ 
 */
/**
 * @brief Ironlink USB object 
 * 
 */
typedef struct il_uart_object{
	il_uart_handle_typedef* uart_port;						//!<Pointer to HAL UART Handle object
	il_buffer_typedef buffer;								//!<Buffer passed to UART peripherial for transmit or recieve functions 
}il_uart_object;
/**@} */

/**@} */

/**
 * @ingroup  Ironlink_USB
 * @{ 
 */
/**
 * @defgroup  Ironlink_USB_Function_Pointers
 * @brief Ironlink USB Function pointers 
 * Created to map the HAL USB library functions
 * to functions in the Ironlink Library 
 * @{ 
 */
typedef void (*il_usb_init_typedef)();												//!<Create Ironlink alias for HAL USB init function
typedef uint8_t (*il_usb_transmit_typedef)(uint8_t*,uint16_t);						//!<Create Ironlink alias for HAL USB transmit function
typedef int8_t (*il_usb_receive_typedef)(uint8_t*,uint32_t);						//!<Create Ironlink alias for HAL USB receive function
/**@} */

/**
 * @defgroup  Ironlink_USB_Objects
 * @brief Ironlink USB struct
 * Buffers and varaibles used to track the status and
 * send data between the USB peripherial and the system memory
 * @{ 
 */
typedef struct il_usb_object {
	il_buffer_typedef buffer;														//!<Buffer passed to USB peripherial for transmit or recieve functions  
}il_usb_object;
/**@} */

/**@} */


/**
 * @ingroup  Ironlink_LORA
 * @{ 
 */

/**
 * @brief LoRa Transmission Frequency Enumeration
 */

 typedef enum
 {
	 IL_LORA_433  				= 433000000U,	//!< 433Mhz Transmission
	 IL_LORA_868				= 868000000U,	//!< 868Mhz Transmission
	 IL_LORA_915				= 915000000U,	//!< 915Mhz Transmission
 } il_lora_frequency;
/**
 * @brief LoRa Modem Type Enumeration
 */
 typedef enum
 {
	RN2483  		= 0x00U,	//!< RN2483 Lora Modem 868Mhz Version
	RN2903		= 0x01U,	//!< RN2903 Lora Modem 915Mhz Version
 } il_lora_modem_type;


/**
 * @brief Lora variable state e.g. on/off
 */
 typedef enum
 {
 	IL_LORA_OFF  			= 0x00U,	//!< Lora variable state ON
 	IL_LORA_ON				= 0x01U,	//!< Lora variable state OFF
 } il_lora_state;


/**
 * @brief LoRa connection mode
 */
 typedef enum
 {
	 IL_LORA_OTA				= 0x00U,    //!< Over the air activation mode
	 IL_LORA_ABP				= 0x01U,    //!< Activation by personalisation mode
	 IL_LORA_RAW_RADIO			= 0x02U		//!< Raw radio mode
 }il_lora_mode;

/**
 * @brief LoRa Modem Configuration Structure
 */

 typedef struct
 {

 	uint8_t 			macEnable;			//!< Enable the WAN stack, RAW radio mode when disabled
 	il_lora_frequency 	frequency;			//!< Set the transmission frequency

 	char 				*appEui;			//!< LoRaWAN Application EUI
 	char 				*appKey;			//!< LoRaWAN Application Key
 	char 				*hweui;				//!< LoRaWAN Hardware EUI

 	uint8_t				transmissionPower; 	//!< LoRa Transmission Power 0-5 for 433Mhz, 1-5 for 868Mhz
 	uint8_t				dataRate; 			//!< Sets the transmission data rate if ADR is enabled
 	uint8_t				adaptiveRate;		//!< Enable/Disable adaptive data rate
 	uint8_t				autoReplies;		//!< Enable/Disable automatic replies

 	il_lora_mode		joinMode;			//!< Modem join mode
 	il_lora_modem_type  modelnumber;		//!< Modem model number

 }il_lora_config;



/**
 * @brief LoRa Modem Packet
 */

 typedef struct
 {

 	uint8_t 	port;						//!< Port of the packet
 	char		payload[30];				//!< Packet payload

 }il_lora_packet;

/**
 * @brief LoRa Modem Status Enumeration
 */
typedef enum
{
  IL_LORA_OK  					= 0x00U,    //!< Modem operation was successful
  IL_LORA_ERROR    				= 0x01U,	//!< A unknown modem error has occurred
  IL_LORA_INVALID_PARAM			= 0x02U,  	//!< Provided parameters are not valid
  IL_LORA_NOT_JOINED 			= 0x03U,   	//!< LoRaWAN transmission error because you have not joined the network
  IL_LORA_NO_CHANNELS_FREE		= 0x04U,	//!< No available channels with the current frequency
  IL_LORA_SILENT				= 0x05U,    //!< HAL_LORA_SILENT 
  IL_LORA_FRAME_COUNTER_ERR		= 0x06U,	//!< HAL_LORA_FRAME_COUNTER_ERR
  IL_LORA_BUSY					= 0x07U,    //!< LoRa modem is busy
  IL_LORA_MAC_PAUSED			= 0x08U,    //!< HAL_LORA_MAC_PAUSED
  IL_LORA_MAC_ERR				= 0x09U,    //!< HAL_LORA_MAC_ERR
  IL_LORA_ACCEPTED				= 0x0AU,	//!< Request was accepted by the network
  IL_LORA_DENIED				= 0x0BU,	//!< Request was denied by the network
  IL_LORA_KEY_NOT_INIT			= 0x0CU,	//!< LoRaWAN keys have not been setup
  IL_LORA_MAC_TX_OK				= 0x0DU,	//!< Transmit ok
  IL_LORA_MAC_RX				= 0x0EU		//!< Receive ok
} il_lora_status;

/**@} */

/********************************** Function maps ****************************/

// DMA Function Maps
/**
 * @ingroup  Ironlink_DMA
 * @{ 
 */
__weak il_dma_init_typedef il_dma_init = MX_DMA_Init;
/**@} */

// Sysytem Clock Function Maps
/**
 * @ingroup  Ironlink_General
 * @{ 
 */
__weak il_get_tick_typedef il_get_tick = HAL_GetTick;


// Delays Function Maps

//<!* Switch between hal and freertos system delays
#ifdef FREERTOS
__weak il_os_delay_typedef il_delay = osDelay;
#else
__weak il_delay_typedef il_delay = HAL_Delay;
#endif
/**@} */

// GPIO Function Map
/**
 * @ingroup  Ironlink_GPIO
 * @{ 
 */
/**
 * @ingroup  Ironlink_GPIO_Function_Pointers
 * @{ 
 */
__weak il_gpio_init_typedef il_gpio_init = MX_GPIO_Init;
__weak il_hal_gpio_read_pin_typedef il_hal_gpio_read_pin = HAL_GPIO_ReadPin;
__weak il_hal_gpio_write_pin_typedef il_hal_gpio_write_pin = HAL_GPIO_WritePin;
/**@} */
/**@} */

// ADC Function Maps
/**
 * @ingroup  Ironlink_ADC
 * @{ 
 */
/**
 * @ingroup  Ironlink_ADC_Function_Pointers
 * @{ 
 */
__weak il_adc_handle_typedef *il_adc = &hadc;
__weak il_adc_init_typedef il_hal_adc_init  = MX_ADC_Init;
__weak il_adc_start_typedef il_hal_adc_start = HAL_ADC_Start;
__weak il_adc_get_value_typedef il_hal_adc_get_value = HAL_ADC_GetValue;
__weak il_adc_ready_typedef il_hal_adc_ready = HAL_ADC_PollForConversion;

__weak il_adc_start_dma_typedef il_hal_adc_start_dma = HAL_ADC_Start_DMA;
__weak il_adc_stop_typedef il_hal_adc_stop = HAL_ADC_Stop;
__weak il_adc_stop_typedef il_hal_adc_stop_dma = HAL_ADC_Stop_DMA;

__weak il_adc_calibration_typedef il_adc_calibration = HAL_ADCEx_Calibration_Start;
/**@} */
/**@} */


// I2C Function Maps
/**
 * @ingroup  Ironlink_I2C
 * @{ 
 */
/**
 * @ingroup  Ironlink_I2C_Function_Pointers
 * @{ 
 */
__weak il_i2c_handle_typedef *il_i2c1 = &hi2c1;
__weak il_i2c_handle_typedef *il_i2c2 = &hi2c2;

__weak il_i2c_init_typedef il_i2c1_init = MX_I2C1_Init;
__weak il_i2c_init_typedef il_i2c2_init = MX_I2C2_Init;

__weak il_i2c_ready_typedef il_i2c_ready = HAL_I2C_IsDeviceReady;
__weak il_i2c_get_state_typedef il_i2c_get_state = HAL_I2C_GetState;
__weak il_i2c_get_error_typedef il_i2c_get_error = HAL_I2C_GetError;

__weak il_i2c_write_typedef il_hal_i2c_write = HAL_I2C_Master_Transmit;
__weak il_i2c_write_reg_typedef il_hal_i2c_write_reg = HAL_I2C_Mem_Write;
__weak il_i2c_write_it_typedef il_hal_i2c_write_it = HAL_I2C_Master_Transmit_IT;
__weak il_i2c_write_reg_it_typedef il_hal_i2c_write_reg_it = HAL_I2C_Mem_Write_IT;

__weak il_i2c_read_typedef il_hal_i2c_read = HAL_I2C_Master_Receive;
__weak il_i2c_read_reg_typedef il_hal_i2c_read_reg = HAL_I2C_Mem_Read;
__weak il_i2c_read_it_typedef il_hal_i2c_read_it = HAL_I2C_Master_Receive_IT;
__weak il_i2c_read_reg_it_typedef il_hal_i2c_read_reg_it = HAL_I2C_Mem_Read_IT;
/**@} */
/**@} */

// UART Function Maps
/**
 * @ingroup  Ironlink_UART
 * @{ 
 */
/**
 * @ingroup  Ironlink_UART_Function_Pointers
 * @{ 
 */
__weak il_uart_handle_typedef *il_uart1 = &huart1;
__weak il_uart_handle_typedef *il_uart2 = &huart4;
__weak il_uart_handle_typedef *il_gps_uart = &huart2;
__weak il_uart_handle_typedef *il_modem_uart = &huart3;

__weak il_uart_init_typedef il_hal_uart1_init = MX_USART1_UART_Init;
__weak il_uart_init_typedef il_hal_uart2_init = MX_USART4_UART_Init;
__weak il_uart_init_typedef il_hal_uart_gps_init = MX_USART2_UART_Init;
__weak il_uart_init_typedef il_hal_uart_modem_init = MX_USART3_UART_Init;

__weak il_uart_get_state_typedef il_hal_uart_get_state = HAL_UART_GetState;
__weak il_uart_get_error_typedef il_hal_uart_get_error = HAL_UART_GetError;

__weak il_uart_write_typedef il_hal_uart_write = HAL_UART_Transmit;
__weak il_uart_read_typedef il_hal_uart_read = HAL_UART_Receive;

__weak il_uart_write_it_typedef il_hal_uart_write_it = HAL_UART_Transmit_IT;
__weak il_uart_read_it_typedef il_hal_uart_read_it = HAL_UART_Receive_IT;

__weak il_uart_abort_typedef il_hal_uart_abort = HAL_UART_Abort;
__weak il_uart_abort_typedef il_hal_uart_abort_write = HAL_UART_AbortTransmit;
__weak il_uart_abort_typedef il_hal_uart_abort_read = HAL_UART_AbortReceive;
__weak il_uart_abort_typedef il_hal_uart_abort_it = HAL_UART_Abort_IT;
__weak il_uart_abort_typedef il_hal_uart_abort_write_it = HAL_UART_AbortTransmit_IT;
__weak il_uart_abort_typedef il_hal_uart_abort_read_it = HAL_UART_AbortReceive_IT;
/**@} */
/**@} */

// USB Function Maps
/**
 * @ingroup  Ironlink_USB
 * @{ 
 */
/**
 * @ingroup  Ironlink_USB_Function_Pointers
 * @{ 
 */
__weak il_usb_init_typedef il_usb_init = MX_USB_DEVICE_Init;
__weak il_usb_transmit_typedef il_usb_transmit = CDC_Transmit_FS;
/**@} */
/**@} */

/********************************** Enumerations ****************************/

/**
 * @ingroup  Ironlink_General
 * @{ 
 */
enum
{
	OFF								= 0x00U,
	ON								= 0x01U

};
/**@} */


/**
 * @ingroup  Ironlink_I2C
 * @{ 
 */

enum{
	IL_I2C_CHANNEL_1 				= 0x00U,
	IL_I2C_CHANNEL_2 				= 0x01U,
};
/**@} */
/************************************ Externs *******************************/

/**
 * @ingroup  Ironlink_I2C
 * @{ 
 */

/**
 * @defgroup Ironlink_I2C_Externs
 * @{ 
 */

#ifdef IL_I2C_OBJECTS_PREDEFINED
extern il_i2c_object il_i2c_bus_1;
extern il_i2c_object il_i2c_bus_2;
#endif
/**@} */

/**@} */

/**
 * @ingroup  Ironlink_UART
 * @{ 
 */
/**
 * @defgroup Ironlink_UART_Externs
 * @{ 
 */

#ifdef IL_UART_OBJECTS_PREDEFINED
	extern il_uart_object modem;
	extern il_uart_object gps;
	extern il_uart_object uart1;
	extern il_uart_object uart2;
#endif
/**@} */
/**@} */

/**
 * @ingroup  Ironlink_USB
 * @{ 
 */
/**
 * @defgroup Ironlink_USB_Externs
 * @{ 
 */
#ifdef IL_USB_OBJECTS_PREDEFINED
	extern il_usb_object usb;
#endif
/**@} */
/**@} */

/******************************* Function Prototypes **************************/

/*
 * General Function Prototypes
 */
/**
 * @ingroup  Ironlink_General
 * @{ 
 */
/**
 * @defgroup Ironlink_General_Function_Prototypes
 * @{ 
 */

il_status_typedef il_copy_buffer(uint8_t *source_buffer, uint8_t *target_buffer, size_t buffer_size);
void il_clear_buffer( void *buffer,size_t buffer_size);
void il_binary_to_string(uint32_t value, char *binarystring);
void il_printf(const char *buff, ...);
/**@} */
/**@} */

/**
 * @ingroup  Ironlink_GPIO
 * @{ 
 */
/**
 * @defgroup Ironlink_GPIO_Function_Prototypes
 * @{ 
 */
uint8_t il_gpio_read_pin(uint16_t pin_name);
void il_gpio_write_pin(uint16_t pin_name, il_gpio_pinstate pin_state);
void il_gpio_toggle_pin(uint16_t pin_name, uint16_t toggle_delay);

/**@} */
/**@} */

/**
 * @ingroup  Ironlink_ADC
 * @{ 
 */
/**
 * @defgroup Ironlink_ADC_Function_Prototypes
 * @{ 
 */
void il_adc_init(void);
/**@} */
/**@} */

/**
 * @ingroup  Ironlink_I2C
 * @{ 
 */
/**
 * @defgroup Ironlink_I2C_Function_Prototypes
 * @{ 
 */
void il_i2c_init(void);
void il_i2c_map(void);
il_status_typedef il_i2c_write_struct(il_i2c_object *il_i2c);
il_status_typedef il_i2c_read_struct(il_i2c_object *il_i2c);
il_status_typedef il_i2c_read_register_struct(il_i2c_object *il_i2c);
void il_i2c_set_register8(il_i2c_object *il_i2c, uint8_t reg);
void il_i2c_set_register16(il_i2c_object *il_i2c, uint16_t reg);
/**@} */
/**@} */

/*
 * UART Function Prototypes
 */
/**
 * @ingroup  Ironlink_UART
 * @{ 
 */
/**
 * @defgroup Ironlink_UART_Function_Prototypes
 * @{
 */
void il_uart_set_sol_n_eol_chars(il_uart_object *il_uart, char startofline_char, char endofline_char);
void il_uart_set_eol_char(il_uart_object *il_uart, char endofline_char);
void il_uart_set_sol_char(il_uart_object *il_uart, char startofline_char);

void il_uart_init(void);
void il_uart_port_map();
il_status_typedef il_uart_write_line(il_uart_object *il_uart, char *message, uint32_t timeout);
il_status_typedef il_uart_read_line(il_uart_object *il_uart, uint8_t *buffer, uint32_t timeout);
il_status_typedef il_uart_read_line_old(il_uart_object *il_uart, uint8_t *buffer, uint32_t timeout);
//uint8_t il_uart_read_line_gps(il_uart_object *il_uart, uint8_t *buffer, uint32_t timeout);
//HAL_StatusTypeDef IL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

// UART Interrupt Functions
void il_uart_flush_buffer(il_uart_object *il_uart);
void il_uart_readline_IRQ(il_uart_object *il_uart);
void il_uart_isr_receive(il_uart_object *il_uart, uint32_t interrupt_status_register_flags);
void il_uart_isr_receive_end(il_uart_object *il_uart);
void il_uart_isr_transmit(il_uart_object *il_uart);
void il_uart_isr_transmit_end(il_uart_object *il_uart);
void il_uart_isr_tx_complete_callback(il_uart_handle_typedef *il_uart);
void il_uart_irq_handler(il_uart_object *il_uart);
/**@} */
/**@} */

/**
 * @ingroup  Ironlink_USB
 * @{ 
 */
/**
 * @defgroup Ironlink_USB_Function_Prototypes
 * @{ 
 */
void il_usb_set_sol_n_eol_chars(il_usb_object *il_usb, char startofline_char, char endofline_char);
void il_usb_set_eol_char(il_usb_object *il_usb, char endofline_char);
void il_usb_set_sol_char(il_usb_object *il_usb, char startofline_char);

void il_usb_write_line(il_usb_object *il_usb, char* tx_buffer);
void il_usb_read_line(il_usb_object *il_usb, char* rx_buffer);
void il_usb_read_irq_handler_keyboard(il_usb_object *il_usb, uint8_t *Buf);

/**@} */
/**@} */

/**
 * @ingroup  Ironlink_LORA
 * @{ 
 */
/**
 * @defgroup Ironlink_LORA_Function_Prototypes
 * @{ 
 */
/**
 * @brief Physically Reset The LoRa Modem
 */
void il_lora_disable_modem();

/**
 * @brief Physically Enable The LoRa Modem And Check For Boot Message
 * @return #IL_LORA_STATUS
 */
il_lora_status il_lora_enable_modem(il_lora_modem_type LORATYPE);

il_lora_status il_lora_modem_send_command(char *command, char *value, uint16_t timeout);

il_lora_status il_lora_modem_read_command(char *command, char *value, char* return_buffer, uint16_t return_buffer_size, uint16_t timeout);

il_lora_status il_lora_modem_default_config(il_lora_config *lora_config, il_lora_modem_type LORATYPE);

il_lora_status il_lora_modem_get_hardware_eui(il_lora_config *config); 

il_lora_status il_lora_modem_init(il_lora_config *config);

il_lora_status il_lora_modem_parse_response(char* response);

il_lora_status il_lora_modem_join_network(il_lora_config *config);

il_lora_status il_lora_modem_send_packet_u32(uint32_t value, uint8_t port, char* packet_type);



/**@} */
/**@} */

#endif /* INC_IRONLINK_IRONLINK_LIBRARY_H_ */
