/**
 * @file ironlink-library.c
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
 */


#include "ironlink-library.h"

/*
 * 		Predefined Objects
 */
#ifdef IL_I2C_OBJECTS_PREDEFINED
il_i2c_object il_i2c_bus_1;
il_i2c_object il_i2c_bus_2;
#endif

#ifdef IL_UART_OBJECTS_PREDEFINED
	il_uart_object modem;
	il_uart_object gps;
	il_uart_object uart1;
	il_uart_object uart2;
#endif

#ifdef IL_USB_OBJECTS_PREDEFINED
	il_usb_object usb;
#endif


/**
 * @ingroup  Ironlink_General
 * @{ 
 */

/**
 * @brief Function to copy one buffer to another
 * 
 * @param source_buffer 			Pointer to buffer to be copied from 
 * @param target_buffer 			Pointer to buffer to be copied to 
 * @param buffer_size 				Buffer size to be copied
 * @return il_status_typedef 
 */
il_status_typedef il_copy_buffer(uint8_t *source_buffer, uint8_t *target_buffer, size_t buffer_size)
{
	uint8_t mem_check = 0;

	// Copy the source buffer into the target buffer
	memcpy(target_buffer, source_buffer, buffer_size);

	// Set the last char in the at the end of the message to 0,
	// Important if message is shorter than the max array size being passed
	target_buffer[buffer_size] = 0;

	// Check if the target_buffer is the same as the source_buffer
	mem_check = memcmp(target_buffer, source_buffer, buffer_size);

	// If the two buffers aren't the same return error
	if (mem_check != 0)
	{
		return IL_ERROR;
	}

	return IL_OK;
}
/**
 * @brief Function to set the values in a buffer to '\0'
 * 
 * @param buffer 			Pointer to buffer to be cleared 
 * @param buffer_size 		Size of buffer to be cleared 
 */

void il_clear_buffer( void *buffer,size_t buffer_size)
{
	memset(buffer, '\0', buffer_size);
}
/**
 * @brief Function to convert a value into a binary string
 * 
 * @param value					value to be converted into binary
 * @param binarystring 			Pointer to buffer to store binary string	
 */
void il_binary_to_string(uint32_t value, char *binarystring)
{
	for(int i=0; i<=32; i++)
	{
		if(value & (1<<i))
		{
			binarystring[i] = '1';
		}
		else
		{
			binarystring[i] = '0';
		}
	}
}
/**
 * @brief Ironlink printf function to send strings via the usb terminal 
 * @note Internal buffer size 256 bytes 
 * @param buff Format string to be parsed 
 * @param ...  Arguments to go in the format string  
 */
void il_printf(const char *buff, ...) {
    // Create buffer to be sent
	char buffer[256];
	// Create list of args from inputs
    va_list args;
    va_start(args, buff);
	// Use the format string and args to create the string to be sent
    vsnprintf(buffer, sizeof(buffer)-1, buff, args);
	// Transmit the string
    il_usb_transmit((uint8_t*)buffer, strlen(buffer));
	// Clean up args
    va_end(args);
}
/**@} */

/**
 * @ingroup  Ironlink_GPIO
 * @{ 
 */

/**
 * @brief Read the pin_state to the chosen GPIO
 * 
 * @param pin_name Pin name to be read, it can be GPIO1 - GPIO7
 * @return uint8_t 
 */
uint8_t il_gpio_read_pin(uint16_t pin_name)
{
	GPIO_TypeDef *GPIO_PORT;
	uint8_t pin_state = 0;


	if(pin_name == GPIO1 || pin_name == GPIO2)
	{
		GPIO_PORT = GPIOA;
	}
	else if(pin_name == GPIO3 || pin_name == GPIO4 || pin_name == GPIO5 || pin_name == GPIO7 )
	{
		GPIO_PORT = GPIOB;
	}

	pin_state = il_hal_gpio_read_pin(GPIO_PORT, pin_name);
	return pin_state;
}

/**
 * @brief Write the pin_state to the chosen GPIO
 * 
 * @param pin_name Pin name to be written to, can be GPIO1 - GPIO7
 * @param pin_state State to set the pin to, can be SET or RESET
 */
void il_gpio_write_pin(uint16_t pin_name, GPIO_PinState pin_state)
{
	GPIO_TypeDef *GPIO_PORT;


	if(pin_name == GPIO1 || pin_name == GPIO2)
	{
		GPIO_PORT = GPIOA;
	}
	else if(pin_name == GPIO3 || pin_name == GPIO4 || pin_name == GPIO5 || pin_name == GPIO7 || pin_name == IL_LORA_MODEM_RST_PIN)
	{
		GPIO_PORT = GPIOB;
	}

	il_hal_gpio_write_pin(GPIO_PORT, pin_name, pin_state);
}


/**
 * @brief Toggle the pin between SET and RESET
 * 
 * @param pin_name Pin name to be toggled, it can be GPIO1 - GPIO7
 * @param toggle_delay delay between switching pin state
 */
void il_gpio_toggle_pin(uint16_t pin_name, uint16_t toggle_delay)
{
	GPIO_TypeDef *GPIO_PORT;


	if(pin_name == GPIO1 || pin_name == GPIO2)
	{
		GPIO_PORT = GPIOA;
	}
	else if(pin_name == GPIO3 || pin_name == GPIO4 || pin_name == GPIO5 || pin_name == GPIO7 )
	{
		GPIO_PORT = GPIOB;
	}

	il_hal_gpio_write_pin(GPIO_PORT, pin_name, GPIO_PIN_SET);

	il_delay(toggle_delay);

	il_hal_gpio_write_pin(GPIO_PORT, pin_name, GPIO_PIN_RESET);

}
/**@} */


/**
 * @ingroup  Ironlink_ADC
 * @{ 
 */

/**
  * @brief Initialisation function for the ADC Peripherial
  * 		If defined in the header file auto start the ADC peripheral.
  * @params none
  * @retval none
  */
void il_adc_init(void)
{
	il_hal_adc_init();
}
/**@} */

/**
 * @ingroup  Ironlink_I2C
 * @{ 
 */
/**
  * @brief Initialisation function for the I2C Peripherials
  * @params none
  * @retval none
  */
void il_i2c_init(void)
{
	il_i2c1_init();
	il_i2c2_init();
#ifdef IL_I2C_OBJECTS_PREDEFINED
	il_i2c_map();
#endif
}

/**
  * @brief Map the I2C Peripherials to the i2c channel object
  * @params none
  * @retval none
  */
void il_i2c_map(void)
{
	il_i2c_bus_1.i2c_channel = il_i2c1;
	il_i2c_bus_2.i2c_channel = il_i2c2;
}

/**
 * @brief Sets the 8 bit I2C register address.
 * 
 * @param il_i2c I2C object
 * @param reg the register address to be set.
 */
void il_i2c_set_register8(il_i2c_object *il_i2c, uint8_t reg)
{
	il_i2c->register_size = 1;
	il_i2c->register_address[0] = reg;
}
/**
 * @brief ets the 16 bit I2C register address.
 * 
 * @param il_i2c I2C object
 * @param reg the register address to be set.
 */
void il_i2c_set_register16(il_i2c_object *il_i2c, uint16_t reg)
{
	il_i2c->register_size = 2;
	il_i2c->register_address[0] = reg >> 8;
	il_i2c->register_address[1] = reg & 0x00FF;
}
// TODO: NEEEDS TESTING
/**
 * @brief Write I2C Data from the peripherials
 * 
 * @param il_i2c ironlink i2c object for uart communcation
 * @return il_status_typedef 
 */
il_status_typedef il_i2c_write_struct(il_i2c_object *il_i2c)
{
	il_status_typedef status = IL_OK;					// Make sure status is in a defined state
#ifdef IL_I2C_INTERRUPT_MODE							// Defined in the header file

	// Check if there is a register you want to write to
	if (il_i2c->register_address != 0)
	{
		// if there is send the register address to the device
		status = il_hal_i2c_write_it(il_i2c->i2c_channel, il_i2c->device_address, (uint8_t*)&il_i2c->register_address, 1);
		// Wait for the operation to be completed.
		while(il_i2c->i2c_channel->State != HAL_I2C_STATE_READY)
		{
			il_delay(1);
		}

		//Make sure the write operation was successful
		if(status != IL_OK)
		{
			// If not exit and return the status
			return status;
		}
	}
	// Check if there is anything else to transmit
	if (il_i2c->buffer.tx_length == 0)
	{
		// If not exit function
		return 0;
	}

	// Send the data in the transmit buffer
	status = il_hal_i2c_write_it(il_i2c->i2c_channel, il_i2c->device_address, (uint8_t*)il_i2c->buffer.tx_buffer, il_i2c->buffer.tx_length);

	while(il_i2c->i2c_channel->State != HAL_I2C_STATE_READY)
	{
		il_delay(1);
	}
// TODO: NEEEDS TESTING
#else
	// Check if there is a register you want to write to
	if (il_i2c->register_address != 0)
	{
		// if there is send the register address to the device
		status = il_hal_i2c_write(il_i2c->i2c_channel, il_i2c->device_address, (uint8_t*)&il_i2c->register_address, 1, il_i2c.buffer.timeout);
		//Make sure the write operation was successful
		if(status != IL_OK)
		{
			// If not exit and return the status
			return status;
		}
	}
	// Check if there is anything else to transmit
	if (il_i2c->buffer.tx_length == 0)
	{
		// If not exit function
		return 0;
	}

	// Send the data in the transmit buffer
	status = il_hal_i2c_write(il_i2c->i2c_channel, il_i2c->device_address, (uint8_t*)il_i2c->buffer.tx_buffer, il_i2c->buffer.tx_length, il_i2c.buffer.timeout);

#endif
	return status;
}

 
// TODO: NEEEDS TESTING
/**
 * @brief Read I2C Data from the peripherials
 * 
 * @param il_i2c 
 * @return il_status_typedef 
 */
il_status_typedef il_i2c_read_struct(il_i2c_object *il_i2c)
{
//	uint8_t timeout = il_i2c->buffer.timeout;

	il_status_typedef status = IL_OK;					// Make sure status is in a defined state

	il_clear_buffer(il_i2c->buffer.rx_buffer, IL_I2C_MAX_BUFFER_SIZE);

#ifdef IL_I2C_INTERRUPT_MODE							// Defined in the header file

	status = il_hal_i2c_read_it(il_i2c->i2c_channel, il_i2c->device_address, (uint8_t*)il_i2c->buffer.rx_buffer, il_i2c->buffer.rx_length);

	while(il_i2c->i2c_channel->State != HAL_I2C_STATE_READY )
	{
		// Read from the i2c peripheral if there is data to be read

			il_delay(1);
	}


// TODO: NEEEDS TESTING
#else
//	while(il_i2c->buffer.rx_buffer[0] == 0 && timeout--)
//	{

	// Read from the i2c peripheral if there is data to be read
	il_hal_i2c_read(il_i2c->i2c_channel, il_i2c->device_address, (uint8_t*)il_i2c->buffer.rx_buffer, il_i2c->buffer.rx_length, il_i2c.buffer.timeout);
	il_delay(1);

//	}
	if (timeout == 0)
	{
		return IL_TIMEOUT;
	}

#endif
	return status;
}


/**
 * @brief Read I2C Data from the peripherials
 * 
 * @param il_i2c ironlink i2c object for uart communcation
 * @return il_status_typedef 
 */
il_status_typedef il_i2c_read_register_struct(il_i2c_object *il_i2c)
{
	uint8_t status = IL_OK;

	// Make sure the transmit buffer is 0 to ensure just the register is written to the device
	il_i2c->buffer.tx_length = 0;

	// Make sure the register address is defined, if it isn't exit the function with error status
	if (il_i2c->register_address == 0)
	{
		return IL_ERROR;
	}

	// Write to the device the register to be written to.
	status = il_i2c_write_struct(il_i2c);

	// Make sure the operation was successful if not exit function with the status.
	if(status != IL_OK)
	{
		return status;
	}

	status = il_i2c_read_struct(il_i2c);

	return status;
}
/**@} */


/**
 * @ingroup  Ironlink_UART
 * @{ 
 */

/**
  * @brief Function to initialise the uart bus
  * @param None
  * @retval None
  */
void il_uart_init(void)
{
	  il_hal_uart1_init();
	  il_hal_uart2_init();
	  il_hal_uart_modem_init();
	  il_hal_uart_gps_init();
#ifdef IL_UART_OBJECTS_PREDEFINED
	  il_uart_port_map();
#endif
}

/**
  * @brief Map the HAL uart object to the ironlink uart objects
  * @param None
  * @retval None
  */
void il_uart_port_map(void)
{
	gps.uart_port = il_gps_uart;
	uart1.uart_port = il_uart1;
	uart2.uart_port = il_uart2;
	modem.uart_port = il_modem_uart;

}




/**
 * @brief Send a null terminated string using the uart peripheral, interrupt based
 * @note  Requires interrpts to be setup to work.
 * @note  If timeout keeps triggering make sure tx complete callback is defined.
 * @param il_uart ironlink uart object for uart communcation
 * @param message message to be sent, expecting a string.
 * @param timeout time to wait till message complete.
 * @return il_status_typedef 
 */
il_status_typedef il_uart_write_line(il_uart_object *il_uart, char *message, uint32_t timeout)
{

	il_status_typedef status = IL_OK;

	//Make sure the transmit buffer is clear
	il_clear_buffer(il_uart->buffer.tx_buffer.u8_buffer, MAX_BUFFER);
	// Copy the message to be transmitted to the transmit buffer
	status = il_copy_buffer((uint8_t*)message, il_uart->buffer.tx_buffer.u8_buffer, strlen(message));
	// Calculate the message length, only works with strings that are null terminated
	il_uart->buffer.tx_length = strlen(il_uart->buffer.tx_buffer.ch_buffer);
	// Get timeout value in tick count
	uint32_t t = il_get_tick()+timeout;
	// Send the message using interrupts
	il_hal_uart_write_it(il_uart->uart_port, il_uart->buffer.tx_buffer.u8_buffer, il_uart->buffer.tx_length);
	// Wait for the message to be completed
	while (il_uart->buffer.eot_flag != 1)
	{
		if (t < il_get_tick())
		{
			il_uart->buffer.timeout_flag = 1;
			status = IL_TIMEOUT;
			il_hal_uart_abort_write_it(il_uart->uart_port);
			break;
		}
		il_delay(1);
	}

	// Reset the end of transmission flag
	il_uart->buffer.eot_flag = 0;
	il_uart_flush_buffer(il_uart);
	// If the buffer copied successfully set the status to ok else return error


	return status;
}

/**
  * @brief Read a CR/LF terminated message from the uart peripheral
  * @note  Requires interrpts to be setup to work
  * @param il_uart	ironlink uart object for uart communcation
  * @param message	message to be sent, expecting a string.
  * @param timeout	time to wait till message complete.
  * @note  If timeout keeps triggering make sure tx complete callback is defined.
  * @note  This function is blocking. If end of transmission flag is never set it will get stuck here.
  * @retval status
  */
il_status_typedef il_uart_read_line(il_uart_object *il_uart, uint8_t *buffer, uint32_t timeout)
{
	//TODO: add status struct to il_uart object
	il_status_typedef status = IL_OK;
	// TODO: modify function to return read data length
	uint32_t length = 0; 
	uint32_t t = il_get_tick()+timeout,
			 t1 = 0;
		// Begin the uart read using interupts
		il_hal_uart_read_it(il_uart->uart_port, &il_uart->buffer.rx_buffer.u8_buffer[il_uart->buffer.tail], 1);
		// Wait for read to complete
		while (il_uart->buffer.eot_flag != 1)
		{
			// if timeout is reached set the timeout flag
			if (t < il_get_tick())
			{
				il_uart->buffer.timeout_flag = 1;
				status = IL_TIMEOUT;
			}

			il_delay(1);

		}
		// Sometime the eot might be set before the timeout has been reached so to make sure the 
		// the system waits the full timeout time the below code was added 
		if (t > il_get_tick())
		{
			t1 = t - il_get_tick();
			il_delay(t1);
		}

		// Copy data from the uart object into the buffer passed into this function
		while(il_uart->buffer.head != il_uart->buffer.tail)
		{
			/* Get the current character*/
			char rx = il_uart->buffer.rx_buffer.ch_buffer[il_uart->buffer.head++];
			// Make sure it isnt '\0'
			if (rx != '\0'){
				// Copy current charater to buffer
				*buffer++ = rx;
				// Increment length of response
				length++;
			}

		}

		// Reset the end of transmission and timeout flags;
		il_uart->buffer.eot_flag = 0;
		il_uart->buffer.timeout_flag = 0;
		// Clear the rx and tx buffer
		il_uart_flush_buffer(il_uart);
		return status;
}
/**
 * @brief Function to clear the rx and tx buffers in the uart object as well as resetting the head and tail variables 
 * 
 * @param il_uart ironlink uart object for uart communcation
 */
void il_uart_flush_buffer(il_uart_object *il_uart)
{
		// Clear tx and rx buffer buffers
		il_clear_buffer(il_uart->buffer.rx_buffer.ch_buffer, MAX_BUFFER);
		il_clear_buffer(il_uart->buffer.tx_buffer.ch_buffer, MAX_BUFFER);
		// Reset head and tail 
		il_uart->buffer.head = 0;
		il_uart->buffer.tail = 0;
}

//il_uart_error_callback(il_uart_handle_typedef *il_uart)
//{
//
//}

// TODO: Test modification
/**
  * @brief Interrupt handler for the uart peripheral
  * @param il_uart ironlink uart object for uart communcation
  * @note needs to be placed in the stm32f0xx_it.c file
  * @retval None
  */
void il_uart_irq_handler(il_uart_object *il_uart)
{
	// Get the current status of the uart peripherals
	  uint32_t interrupt_status_register_flags = READ_REG(il_uart->uart_port->Instance->ISR);
	  uint32_t control_register_1_interrupts   = READ_REG(il_uart->uart_port->Instance->CR1);

	  /*
	   * If the recevie buffer not empty interrupt fires execute the receive isr
	   */
	  if (((interrupt_status_register_flags & USART_ISR_RXNE) != 0U)
	      && ((control_register_1_interrupts & USART_CR1_RXNEIE) != 0U))
	  {
		  il_uart_isr_receive(il_uart, interrupt_status_register_flags);
	  }
	  /*
	   * Run this if the overrun flag is set but the frame error flag isn't set
	   */
	  if((interrupt_status_register_flags & USART_ISR_ORE) != 0U
			  && (interrupt_status_register_flags & USART_ISR_FE) == 0U )
	  {
		  __HAL_UART_CLEAR_OREFLAG(il_uart->uart_port);
		  il_uart_isr_receive(il_uart, interrupt_status_register_flags);
		  __HAL_UNLOCK(il_uart->uart_port);

	  }
	  /*
	   * Run this if the frame error flag is set but the overrun error flag isn't set
	   */
	  if((interrupt_status_register_flags & USART_ISR_FE) != 0U
			  && (interrupt_status_register_flags & USART_ISR_ORE) == 0U )
	  {
		  __HAL_UART_CLEAR_FEFLAG(il_uart->uart_port);
		  il_uart_isr_receive(il_uart, interrupt_status_register_flags);
		  __HAL_UNLOCK(il_uart->uart_port);

	  }
	  /*
	   * Run this if the frame error flag and the overrun error flag is set
	   */
	  if((interrupt_status_register_flags & USART_ISR_FE) != 0U
	  			  && (interrupt_status_register_flags & USART_ISR_ORE) != 0U )
	  {
		  __HAL_UART_CLEAR_OREFLAG(il_uart->uart_port);
		  __HAL_UART_CLEAR_FEFLAG(il_uart->uart_port);
		  il_uart_isr_receive(il_uart, interrupt_status_register_flags);
		  __HAL_UNLOCK(il_uart->uart_port);

	  }
	  /*
	   * If the recevie buffer not empty interrupt and the RXNEIE flag
	   * has been cleared and run the end of receive function
	   */
	  if (((interrupt_status_register_flags & USART_ISR_RXNE) == 0U)
	        && ((control_register_1_interrupts & USART_CR1_RXNEIE) == 0U))
	  {
		  il_uart_isr_receive_end(il_uart);
	  }
	  /*
	   * If the transmit buffer empty interrupt fires execute the transmit function
	   */
	  if (((interrupt_status_register_flags & USART_ISR_TXE) != 0U)
	      && ((control_register_1_interrupts & USART_CR1_TXEIE) != 0U))
	  {
		  il_uart_isr_transmit(il_uart);
	  }
	  /*
	   * If the transmit complete interrupt fires execute the transmit end function
	   */
	  if (((interrupt_status_register_flags & USART_ISR_TC) != 0U) &&
			  ((control_register_1_interrupts & USART_CR1_TCIE) != 0U))
	  {
		  il_uart_isr_transmit_end(il_uart);
	  }
	  return;
}
// TODO: Test modification
/**
 * @brief Interrupt service routine to receive a new character
 * 
 * @param il_uart ironlink uart object for uart communcation
 * @param interrupt_status_register_flags the interrupt status register current values
 */
void il_uart_isr_receive(il_uart_object *il_uart, uint32_t interrupt_status_register_flags)
{
	if(++il_uart->buffer.tail >= MAX_BUFFER)
	{
		il_uart->buffer.tail = 0;
	}
	if ((interrupt_status_register_flags & USART_ISR_RXNE)!=0) // Has a new character been recieved 
	{
			// Read the RDR register which contains the newest character

			il_uart->buffer.rx_char.ch_byte = (uint16_t)((READ_REG(il_uart->uart_port->Instance->RDR)) & (uint8_t)il_uart->uart_port->Mask);

			// Copy the latest char to the il_uart objects rx_char variable

			il_uart->buffer.rx_buffer.u8_buffer[il_uart->buffer.tail] = il_uart->buffer.rx_char.u8_byte;

			if(il_uart->uart_port->RxXferCount > 0U) // If the receive transfer counter is larger than 0
			{
				il_uart->uart_port->RxXferCount--;   // Decrement the receive transfer counter
			}

	}
	// If the UART IDLE interrupt is set and the Rx not empty interrupt is not set, set the end of transmission flag 
	if ((interrupt_status_register_flags & UART_IT_IDLE) !=0 && (interrupt_status_register_flags & USART_ISR_RXNE)!=0)
	{
		il_uart->buffer.eot_flag = 1;
	}

}

/**
 * @brief Interrupt service routine to end the receive transmission
 * 
 * @param il_uart ironlink uart object for uart communcation
 */
void il_uart_isr_receive_end(il_uart_object *il_uart)
{

	  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
	  CLEAR_BIT(il_uart->uart_port->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	  CLEAR_BIT(il_uart->uart_port->Instance->CR3, USART_CR3_EIE);

	  /* At end of Rx process, restore huart->RxState to Ready */
	  il_uart->uart_port->RxState = HAL_UART_STATE_READY;

	  /* Reset RxIsr function pointer */
	  il_uart->uart_port->RxISR = NULL;
	  return;
}


/**
 * @brief Interrupt service routine to transmit a new character
 * 
 * @param il_uart ironlink uart object for uart communcation
 */
void il_uart_isr_transmit(il_uart_object *il_uart)
{

	  if (il_uart->uart_port->gState == HAL_UART_STATE_BUSY_TX)
	  {
	    if (il_uart->uart_port->TxXferCount == 0U)
	    {
	      /* Disable the UART Transmit Data Register Empty Interrupt */
	      CLEAR_BIT(il_uart->uart_port->Instance->CR1, USART_CR1_TXEIE);

	      /* Enable the UART Transmit Complete Interrupt */
	      SET_BIT(il_uart->uart_port->Instance->CR1, USART_CR1_TCIE);
	    }
	    else
	    {
	    	/* Put the first char in the TDR buffer */
			il_uart->uart_port->Instance->TDR = (uint8_t)(*il_uart->uart_port->pTxBuffPtr & (uint8_t)0xFF);
			/* Increment the Transmit Buffer Pointer */
			il_uart->uart_port->pTxBuffPtr++;
			/* Decrement the Tranmit transfer counter  */
			il_uart->uart_port->TxXferCount--;
	    }
	  }
	  return;

}

/**
 * @brief Interrupt service routine to end the transmit transmission
 * 
 * @param il_uart ironlink uart object for uart communcation
 */
void il_uart_isr_transmit_end(il_uart_object *il_uart)
{
	  /* Disable the UART Transmit Complete Interrupt */
	  CLEAR_BIT(il_uart->uart_port->Instance->CR1, USART_CR1_TCIE);

	  /* Transmit process is ended, restore huart->gState to Ready */
	  il_uart->uart_port->gState = HAL_UART_STATE_READY;

	  /* Cleat Transmit ISR function pointer */
	  il_uart->uart_port->TxISR = NULL;

	  /*Call Transmit complete callback*/
	  il_uart_isr_tx_complete_callback(il_uart->uart_port);
	  return;
}

/**
 * @brief Interrupt service routine transmit callback function.
 * 
 * @param il_uart Ironlink uart object for uart communcation
 */
void il_uart_isr_tx_complete_callback(il_uart_handle_typedef *il_uart)
{
	if(il_uart->Instance == USART1)
	{
		uart1.buffer.eot_flag = 1;
	}
	if(il_uart->Instance == USART2)
	{
		gps.buffer.eot_flag = 1;
	}
	if(il_uart->Instance == USART3)
	{
		modem.buffer.eot_flag = 1;
	}
	if(il_uart->Instance == USART4)
	{
		uart2.buffer.eot_flag = 1;
	}
}

/**@} */
/**
  * @brief Callback function to handle end of line conditions on the uart bus
  * @param uart object
  * @retval
  */
//void il_uart_read_callback(il_uart_handle_typedef *il_uart)
//{
//
//
//}


/**
 * @ingroup  Ironlink_USB
 * @{ 
 */
/**
 * @brief Set the start and end of line character for the il_uart object
 *
 * @param il_usb ironlink usb object for usb communcation
 * @param startofline_char charcter used by the interrupt to determine the start of the line
 * @param endofline_char charcter used by the interrupt to determine the end of the line
 */
void il_usb_set_sol_n_eol_chars(il_usb_object *il_usb, char startofline_char, char endofline_char)
{
	il_usb->buffer.sol_char.ch_byte = startofline_char;
	il_usb->buffer.eol_char.ch_byte = endofline_char;
	return;
}

/**
 * @brief Set the start of line character for the il_uart object
 *
 * @param il_usb ironlink usb object for usb communcation
 * @param startofline_char charcter used by the interrupt to determine the start of the line
 */
void il_usb_set_sol_char(il_usb_object *il_usb, char startofline_char)
{
	il_usb->buffer.sol_char.ch_byte = startofline_char;
	return;
}


/**
 * @brief Set the end of line character for the il_uart object
 *
 * @param il_usb ironlink usb object for usb communcation
 * @param endofline_char charcter used by the interrupt to determine the end of the line
 */
void il_usb_set_eol_char(il_usb_object *il_usb, char endofline_char)
{

	il_usb->buffer.eol_char.ch_byte = endofline_char;
	return;
}

/**
 * @brief Function to write message using the USB Peripherial
 * 
 * @param il_usb Ironlink usb object for usb communcation
 * @param tx_buffer Pointer to buffer containing string to be transmitted
 */
void il_usb_write_line(il_usb_object *il_usb, char* tx_buffer)
{
	il_usb->buffer.tx_length = strlen(tx_buffer);
	memcpy((void*)il_usb->buffer.tx_buffer.ch_buffer, (void*)tx_buffer, il_usb->buffer.tx_length);
	il_usb_transmit(il_usb->buffer.tx_buffer.u8_buffer, il_usb->buffer.tx_length);
}

/**
 * @brief Function to read string from the USB Peripherial
 * @note This function is blocking
 * @param il_usb Ironlink usb object for usb communcation
 * @param rx_buffer Pointer to buffer used to recieve message
 */
void il_usb_read_line(il_usb_object *il_usb, char* rx_buffer)
{

	while (il_usb->buffer.eot_flag != 1)
	{
		__NOP();
		osDelay(1);
	}


	memcpy(rx_buffer, il_usb->buffer.rx_buffer.ch_buffer, il_usb->buffer.rx_index);

	memset((void *)il_usb->buffer.rx_buffer.ch_buffer, '\0', MAX_BUFFER);

	il_usb->buffer.eot_flag = 0;

	il_usb->buffer.rx_index = 0;

	return;
}

/**
  * @brief Read Interrupt handler for the usb peripheral
  * @param il_usb_object ironlink usb object for usb communcation
  * 
  * @retval None
  */
/**
 * @brief Read Interrupt handler for the usb peripheral
 * @note needs to be placed in the usbd_cdc_if.c file
 * @param il_usb Ironlink usb object for usb communcation
 * @param Buf Pointer to buffer used to recieve message
 */
//TODO: change Buf to rx_buffer
void il_usb_read_irq_handler_keyboard(il_usb_object *il_usb, uint8_t *Buf)
{
	static uint16_t rx_index = 0;
	// Copy over the current char
	il_usb->buffer.rx_char.ch_byte = Buf[0];

	if ( (il_usb->buffer.rx_char.ch_byte == il_usb->buffer.eol_char.ch_byte)) // Is this an end-of-line condition, either will suffice?
	{
		// Make sure the line has some content
		if (rx_index != 0)
		{
			// Make sure the rx index value is less than max buffer size -1 for padding purposes
			if (rx_index < MAX_BUFFER-1)
				il_usb->buffer.rx_buffer.ch_buffer[rx_index++] = il_usb->buffer.rx_char.ch_byte;
			// Add terminating NUL
			il_usb->buffer.rx_buffer.ch_buffer[rx_index] = '\0';
			// flag new line valid for processing
			il_usb->buffer.eot_flag = 1;
			// Copy message length to index
			il_usb->buffer.rx_index = rx_index;
			// Reset content pointer
			rx_index = 0;

		}
	}
	else
	{
		// TODO: Add optional start of line terminator
		// If resync or overflows pull back to start
		if ((il_usb->buffer.rx_char.ch_byte == il_usb->buffer.sol_char.ch_byte) || (rx_index == MAX_BUFFER))
			// Reset content pointer
			rx_index = 0;
		// Copy to buffer
		il_usb->buffer.rx_buffer.ch_buffer[rx_index++] = il_usb->buffer.rx_char.ch_byte;
	}
	return;
}
/**@} */


/**
 * @ingroup  Ironlink_LORA
 * @{ 
 */

/**
 * @brief Physically Reset The LoRa Modem
 * 
 */
void il_lora_disable_modem(void) {

	// Send a message to via USB that Modem is turning off
	il_usb_transmit((uint8_t*)"Turning Modem Off . . . ", sizeof("Turning Modem Off . . . "));

	// Disble the modem by pulling the reset pin low
	il_gpio_write_pin(IL_LORA_MODEM_RST_PIN, RESET);
}


/**
 * @brief Physically Enable The LoRa Modem And Check For Boot Message
 * 
 * @param LORATYPE Lora modem type RN2483 for 868MHz and RN2903 for 915MHz
 * @return uint8_t 
 */
il_lora_status il_lora_enable_modem(il_lora_modem_type LORATYPE) {

	il_usb_transmit((uint8_t*)"Turning Modem On . . . \r\n", sizeof("Turning Modem On . . . \r\n"));

	il_delay(5);

	uint8_t recv[50] = {'\0'};

	// Enable the modem by pulling the reset pin high
	il_gpio_write_pin(IL_LORA_MODEM_RST_PIN, SET);

	// Needs to be over 1000 millisecs to ensure the message is recieved
	il_uart_read_line(&modem, (uint8_t *)recv, 1000);
	il_delay(5);

	switch(LORATYPE)
	{
		case RN2483:
			if (strstr((char*)recv, "RN2483") != NULL) {

				il_usb_transmit(recv, 50);
				il_delay(5);
				il_usb_transmit((uint8_t*)"BOOT LORA SUCCESSFUL!\r\n", sizeof("BOOT LORA SUCCESSFUL!\r\n"));
				return IL_LORA_OK;
			}
			else {

				// No boot message, put the modem into reset mode
				il_gpio_write_pin(IL_LORA_MODEM_RST_PIN, SET);
				il_usb_transmit((uint8_t*)"BOOT LORA FAILED!\r\n", sizeof("BOOT LORA FAILED!\r\n"));
				il_delay(5);
				NVIC_SystemReset();
				return IL_LORA_ERROR;
			}

		case RN2903:
			if (strstr((char*)recv, "RN2903") != NULL) {

				il_usb_transmit(recv, 50);
				il_delay(5);
				il_usb_transmit((uint8_t*)"BOOT LORA SUCCESSFUL!\r\n", sizeof("BOOT LORA SUCCESSFUL!\r\n"));
				return IL_LORA_OK;
			}
			else {

				// No boot message, put the modem into reset mode
				il_gpio_write_pin(IL_LORA_MODEM_RST_PIN, SET);
				il_usb_transmit((uint8_t*)"BOOT LORA FAILED!\r\n", sizeof("BOOT LORA FAILED!\r\n"));
				NVIC_SystemReset();
				return IL_LORA_ERROR;
			}
		default:
			il_usb_transmit((uint8_t*)"MODE NOT SET!\r\n", sizeof("MODE NOT SET!\r\n"));
			return IL_LORA_ERROR;

	}

}

/**
 * @brief Function to send a command to the lora modem, response from modem is outputted via USB
 * 
 * @param command command to send to modem, reference of allowed command are in the ironlink header file
 * @param value some commands have values to be sent with them, set to NULL if there is no value to be sent
 * @param timeout how long to wait for reply from command.
 * @return il_lora_status 
 */
il_lora_status il_lora_modem_send_command(char *command, char *value, uint16_t timeout) {

	char buffer[50] = {'\0'};
	uint16_t len = 0;

	// Assemble the transmission string
	if (value != NULL) {
		len = sprintf(buffer, "%s %s\r\n", command, value);
		il_usb_transmit((uint8_t*)buffer, len);
	} else {
		len = sprintf(buffer, "%s\r\n", command);
		il_usb_transmit((uint8_t*)buffer, len);
}

	// Send command to the modem
	il_uart_write_line(&modem, (char *)buffer,200);


	// Clear the buffer
	il_clear_buffer(buffer, 50);


	// Wait for the incoming buffer
	il_uart_read_line(&modem,(uint8_t*)buffer, timeout);


	il_usb_transmit((uint8_t*)buffer, sizeof(buffer));
	il_delay(5);

	return IL_LORA_OK;
}
/**
 * @brief Function to send a command to the lora modem, response from modem is outputted via USB and to buffer provided.
 * 
 * @param command command to send to modem, reference of allowed command are in the ironlink header file
 * @param value some commands have values to be sent with them, set to NULL if there is no value to be sent
 * @param return_buffer modem command response buffer. 
 * @param return_buffer_size modem command response buffer size. 
 * @param timeout how long to wait for reply from command.
 * @return il_lora_status 
 */
il_lora_status il_lora_modem_read_command(char *command, char *value, char* return_buffer, uint16_t return_buffer_size, uint16_t timeout) 
{
	char buffer[50] = {'\0'};
	uint16_t len = 0;

	// Assemble the transmission string
	if (value != NULL) {
		len = sprintf(buffer, "%s %s\r\n", command, value);
		il_usb_transmit((uint8_t*)buffer, len);
	} else {
		len = sprintf(buffer, "%s\r\n", command);
		il_usb_transmit((uint8_t*)buffer, len);
}

	// Send command to the modem
	il_usb_transmit((uint8_t*)buffer, len);
	il_delay(5);
	il_uart_write_line(&modem, (char *)buffer,200);


	// Clear the buffer
	il_clear_buffer(buffer, 50);


	// Wait for the incoming buffer
	il_uart_read_line(&modem,(uint8_t*)buffer, timeout);


	il_usb_transmit((uint8_t*)buffer, sizeof(buffer));
	il_delay(5);

	// Copy the response to the response buffer.
	memcpy(return_buffer, buffer, return_buffer_size);
	return_buffer[return_buffer_size] = '\0';

	return IL_LORA_OK;
}

/**
 * @brief Function to configure the default modem parameters 
 * 
 * @param lora_config lora config object containing the config options 
 * @param LORATYPE lora modem model type RN2483 or RN2903
 * @return il_lora_status 
 */
il_lora_status il_lora_modem_default_config(il_lora_config *lora_config, il_lora_modem_type LORATYPE)
{
	switch(LORATYPE)
		{
			case RN2483:
				lora_config->modelnumber 			= RN2483;						// Either RN_ENABLE_RN2483 or RN_ENABLE_RN2903
				lora_config->appEui 				= "0000000000000000";
				lora_config->frequency 				= IL_LORA_868;
				lora_config->adaptiveRate 			= IL_LORA_ON;
				lora_config->autoReplies 			= IL_LORA_OFF;
				lora_config->transmissionPower 		= 1;
				lora_config->dataRate 				= 4;
				lora_config->joinMode 				= IL_LORA_OTA;							// Choose the OTAA join mode. Requires appEui & appKey to be set.

				// Get hardware eui from the RN2483.
				lora_config->hweui = malloc(sizeof(char) * 17);
				il_lora_modem_read_command(RN_SYS_GET_HWEUI, NULL, lora_config->hweui, 16, 10000);
				break;

			case RN2903:
				lora_config->modelnumber 			= RN2903;						// Either RN_ENABLE_RN2483 or RN_ENABLE_RN2903
				lora_config->appEui 				= "0000000000000000";
				lora_config->frequency 				= IL_LORA_915;
				lora_config->adaptiveRate 			= IL_LORA_ON;
				lora_config->autoReplies 			= IL_LORA_OFF;
				lora_config->transmissionPower 		= 1;
				lora_config->dataRate 				= 4;
				lora_config->joinMode 				= IL_LORA_OTA;							// Choose the OTAA join mode. Requires appEui & appKey to be set.

				// Get hardware eui from the RN2483.
				lora_config->hweui = malloc(sizeof(char) * 17);
				il_lora_modem_read_command(RN_SYS_GET_HWEUI, NULL, lora_config->hweui, 16, 10000);
				break;

		}
	return IL_OK;
}
/**
 * @brief Get the hardware eui from the modem
 * 
 * @param config lora config object containing the config options 
 * @return il_lora_status 
 */
il_lora_status il_lora_modem_get_hardware_eui(il_lora_config *config) 
{
	// Get the device address
	il_lora_modem_send_command(RN_SYS_GET_HWEUI, NULL, 200);

	return IL_LORA_OK;
}
/**
 * @brief Initalise the lora modem
 * 
 * @param config il_lora_config struct with the lora modem configuration options
 * @return il_lora_status 
 */
il_lora_status il_lora_modem_init(il_lora_config *config) {

	// Reset the mac layer
	il_lora_modem_send_command(RN_MAC_RESET_CMD, NULL, 200);

	// Set the device address
	il_lora_modem_send_command(RN_MAC_SET_DEV_EUI, config->hweui, 200);

	// Set the network key
	il_lora_modem_send_command(RN_MAC_SET_APP_KEY, config->appKey, 200);

	// Set the EUI
	il_lora_modem_send_command(RN_MAC_SET_APP_EUI, config->appEui, 200);

	// Set the transmission power
	char power = 0;
	itoa(config->transmissionPower, &power, 10);
	il_lora_modem_send_command(RN_MAC_SET_PWRIDX, &power, 200);

	char rate = 0;
	itoa(config->dataRate, &rate, 10);
	il_lora_modem_send_command(RN_MAC_SET_DATARATE, &rate, 200);

	// Disable adaptive data rate
	char adr[4] = {'\0'};
	if(config->adaptiveRate == IL_LORA_ON) {
		sprintf(adr, "on");
	}
	else if(config->adaptiveRate == IL_LORA_OFF) {
		sprintf(adr, "off");
	}
	il_lora_modem_send_command(RN_MAC_SET_ADR_ON_CMD, adr, 200);

	// Disable automatic replies
	char ar[4] = {'\0'};
	if(config->autoReplies == IL_LORA_ON) {
		sprintf(ar, "on");
	}
	else if(config->autoReplies == IL_LORA_OFF) {
		sprintf(ar, "off");
	}
	il_lora_modem_send_command(RN_MAC_SET_AR_ON_CMD, ar, 200);

	return IL_LORA_OK;
}

/**
 * @brief Function to parce response from the modem
 * 
 * @param response String to be parsed
 * @return il_lora_status 
 */
il_lora_status il_lora_modem_parse_response(char* response) {

	if (strstr(response, "mac_tx_ok\r\n") != NULL) {
		return IL_LORA_MAC_TX_OK;
	}

	else if (strstr(response, "accepted\r\n") != NULL) {
		return IL_LORA_ACCEPTED;
	}

	else if (strstr(response, "denied\r\n") != NULL) {
		return IL_LORA_DENIED;
	}

	else if (strstr(response, "invalid_param\r\n") != NULL) {
		return IL_LORA_INVALID_PARAM;
	}

	else if (strstr(response, "keys_not_init\r\n") != NULL) {
		return IL_LORA_KEY_NOT_INIT;
	}

	else if (strstr(response, "no_free_ch\r\n") != NULL) {
		return IL_LORA_NO_CHANNELS_FREE;
	}

	else if (strstr(response, "silent\r\n") != NULL) {
		return IL_LORA_SILENT;
	}

	else if (strstr(response, "busy\r\n") != NULL) {
		return IL_LORA_BUSY;
	}

	else if (strstr(response, "mac_paused\r\n") != NULL) {
		return IL_LORA_MAC_PAUSED;
	}

	else if (strstr(response, "frame_counter_err_rejoin_needed\r\n") != NULL) {
		return IL_LORA_FRAME_COUNTER_ERR;
	}

	else if (strstr(response, "ok\r\n") != NULL) {
		return IL_LORA_OK;
	}

	else {
		return IL_LORA_ERROR;
	}
}
/**
 * @brief Function to join the Lora network 
 * @note This function is blocking
 * @param config il_lora_config struct with the lora modem configuration options
 * @return il_lora_status 
 */
il_lora_status il_lora_modem_join_network(il_lora_config *config) {

	char buffer[50] = {'\0'};
//	uint8_t length = 0;

	while(1) {
		
		//Send join request in over the air activation mode

		il_lora_modem_read_command(RN_JOIN_OTAA_MODE, NULL, buffer, 50, 5000);

		if (strstr(buffer, "ok\r\n") != NULL) {

			il_usb_transmit((uint8_t*)buffer, 50);
			il_delay(5);
		}

		// Wait for network acceptance

		il_clear_buffer(buffer, 50);

		il_uart_read_line(&modem,(uint8_t*)buffer, 10000);

		if (strstr(buffer, "accepted\r\n") != NULL) {
			il_usb_transmit((uint8_t*)buffer, 50);
			il_delay(5);
			break;
		}
		else {
			il_usb_transmit((uint8_t*)buffer, 50);
			il_delay(5);
		}

		il_clear_buffer(buffer, 50);

	}

	return il_lora_modem_parse_response(buffer);
}

/**
 * @brief Function to send data over the LORA network 
 * 
 * @param value Data to be sent, Data Type: 32bit unsigned interger
 * @param port Port to sent the data over 
 * @param packet_type confirmed : RN_MAC_TX_CONFIRMED or unconfirmed : RN_MAC_TX_UNCONFIRMED
 * @return il_lora_status 
 */
il_lora_status il_lora_modem_send_packet_u32(uint32_t value, uint8_t port, char* packet_type) {

	char buffer[50] = {'\0'};
	char value_str[8] = {'\0'};
	uint8_t len = 0;

	itoa(value, value_str, 16);

	while(1) {

		len = sprintf(buffer, "mac tx %s %u %X\r\n", packet_type, port, (unsigned int)value);

		il_usb_transmit((uint8_t*)buffer, len);
		il_uart_write_line(&modem, (char *)buffer, 200);


		il_clear_buffer(buffer, 50);


		il_uart_read_line(&modem,(uint8_t*)buffer, 5000);
		il_usb_transmit((uint8_t*)buffer, 50);


		il_uart_read_line(&modem,(uint8_t*)buffer, 5000);
		il_usb_transmit((uint8_t*)buffer, 50);



		if(strstr(buffer, "mac_tx_ok") != NULL) {
			break;
		}

		il_clear_buffer(buffer, 50);

	}

	return il_lora_modem_parse_response(buffer);
}
/**@} */
