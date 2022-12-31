/*
 * mcp23s08.h
 *
 *  Created on: Dec 31, 2022
 *      Author: mherc
 *
 *  Conn schema:                                          	                 |GND
 *  					  ------------------                 	             |
 * 			SPI2_SCK|----|1  SCK		GP7	|-			   		   		    C|
 * 			SPI2_MOSI|---|2  SI			GP6	|-        	                  ------
 * 			SPI2_MISO|---|3  SO			GP5	|-				    R100 Om B|		|
 * 			OIEXP_CS|----|7  CS			GP4	|-		OIEXP_RST|----===----|  PNP	|
 * 						 |				GP3	|-							 |		|
 * 			GND|---------|4  A1			GP2	|-						 	  ------
 * 			GND|---------|5  A0			GP1	|-						   		E|
 * 						 |				GP0	|-		-------------------------
 * 						 |					|		|	R100 Om
 * 			VDD|--x------|18 VDD	  RESET	|-------x-----===---|VCC
 * 				  | GND|-|9  VSS		INT	|---|NC
 * 				  |		  ------------------
 *                |
 *   GND|----| |---
 *          C100nf
 */

#ifndef INC_MCP23S08_H_
#define INC_MCP23S08_H_

#include <stdbool.h>
#include <stdint.h>

//Register addresses for MCP23S08
#define MCP_IODIR		0x00
#define MCP_IPOL		0x01
#define MCP_GPINTEN		0x02
#define MCP_DEFVAL		0x03
#define MCP_INTCON		0x04
#define MCP_IOCON		0x05
#define MCP_GPPU		0x06
#define MCP_INTF		0x07
#define MCP_INTCAP		0x08
#define MCP_GPIO		0x09
#define MCP_OLAT		0x0A

/*
 * Forces MCP reset (needs reseting pin in STM32)
 * If MCP is connected into power bus, it needs to be reset separately after STM Reset.
 * After reset MCP needs ca. 1500ms to wake up (practical observation)
 * 50ms for filtering capacitor discharge (100nF) and specification timing (30 + 20)
 */
void mcp_init ( void );

/*
 * Sends data to MCP
 * req - register defined above
 * byte - byte of data
 */
void mcp_reg_write( uint8_t reg, uint8_t byte);

/*
 * Reads data from MCP
 * req - register defined above
 *
 * returns register state as uint8_t
 */
uint8_t mcp_reg_read( uint8_t reg );

/*
 * Sets pin logic value
 * pin - pin number (0-7)
 * value - state (true of false)
 */
void mcp_write_pin( int pin, bool value);

/*
 * Reads pin logic value
 * pin - pin number (0-7)
 *
 * returns boolean value
 */
bool mcp_read_pin( int pin );

/*
 * Sets pin as button input (with pullup)
 * pin - pin number (0-7)
 */
void mcp_set_button_pin( int pin );

#endif /* INC_MCP23S08_H_ */
