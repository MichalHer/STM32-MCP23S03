/*
 * mcp23s08.c
 *
 *  Created on: Dec 31, 2022
 *      Author: mherc
 */
#include <stdbool.h>
#include "mcp23s08.h"
#include "spi.h"
#include "gpio.h"

/*
 * Forces MCP reset (needs reseting pin in STM32)
 * If MCP is connected into power bus, it needs to be reset separately after STM Reset.
 * After reset MCP needs ca. 1500ms to wake up (practical observation)
 * 50ms for filtering capacitor discharge (100nF) and specification timing (30 + 20)
 */
void mcp_init( void ){
	HAL_GPIO_WritePin(IOEXP_RST_GPIO_Port, IOEXP_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(IOEXP_RST_GPIO_Port, IOEXP_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(1500);
}

/*
 * Sends data to MCP
 * req - register defined above
 * byte - byte of data
 */
void mcp_reg_write( uint8_t reg, uint8_t byte){
	uint8_t tx[3] = { 0x40, reg, byte };

	HAL_GPIO_WritePin(IOEXP_CS_GPIO_Port, IOEXP_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, tx, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IOEXP_CS_GPIO_Port, IOEXP_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
}

/*
 * Reads data from MCP
 * req - register defined above
 *
 * returns register state as uint8_t
 */
uint8_t mcp_reg_read( uint8_t reg ){
	uint8_t rxtx[3] = { 0x41, reg, 0xFF };

	HAL_GPIO_WritePin(IOEXP_CS_GPIO_Port, IOEXP_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, rxtx, rxtx, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IOEXP_CS_GPIO_Port, IOEXP_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(14);
	return rxtx[2];
}

/*
 * Sets pin logic value
 * pin - pin number (0-7)
 * value - state (true of false)
 */
void mcp_write_pin( int pin, bool value ){
	uint8_t state = mcp_reg_read(MCP_GPIO);

	if( value ) state |= (1<<pin);
	else state &= ~(1<<pin);

	mcp_reg_write(MCP_OLAT, state);
}

/*
 * Reads pin logic value
 * pin - pin number (0-7)
 *
 * returns boolean value
 */
bool mcp_read_pin( int pin ){
	return (bool)( mcp_reg_read(MCP_GPIO) & (1<<pin) );
}

/*
 * Sets pin as button input (with pullup)
 * pin - pin number (0-7)
 */
void mcp_set_button_pin( int pin ){
	uint8_t state = mcp_reg_read(MCP_IODIR);
	state |= (1<<pin);
	mcp_reg_write(MCP_IODIR, state);

	uint8_t pullups = mcp_reg_read(MCP_GPPU);
	pullups |= (1<<pin);
	mcp_reg_write(MCP_GPPU, pullups);
}
