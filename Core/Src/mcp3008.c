/*
 * MCP3008.c
 *
 */

#include "../Inc/mcp3008.h"

/*
 * Set the MISO, MOSI, SCK and CS
 * SPI settings:
 * CPHA = 1 Edge
 * Prescaler = 8
 * First bit = MBS first
 * CPOL = Low
 */
void MCP3008_Init(MCP3008_SPI* spi, SPI_HandleTypeDef* hspi, GPIO_TypeDef* CS_PORT, uint16_t CS_PIN){
  spi->hspi = hspi;
  spi->CS_PORT = CS_PORT;
  spi->CS_PIN = CS_PIN;
}

// Read the channels from 0 to 7
uint16_t MCP3008_Read_Channel(MCP3008_SPI* spi, uint8_t channel){

	// Declare data that we will send
	uint8_t pTxData[3] = {0};
	pTxData[0] = (0x01);		  // start bit

	pTxData[1] = (1 << 7)|			// SGL
				 (0); 	// channel number, D1 and D0 are 0 when selecting channel 0, everything else can just be 0.
	pTxData[2] = 0x00;

	// Data that we will get
	uint8_t pRxData[3] = {0};

	// CS low, Send and receive, CS high
	HAL_GPIO_WritePin(spi->CS_PORT, spi->CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi->hspi, pTxData, pRxData, 3, 10);
	HAL_GPIO_WritePin(spi->CS_PORT, spi->CS_PIN, GPIO_PIN_SET);

	// Compute the ADC
return 0x3FF & ((pRxData[1] & 0xFF) << 8 | (pRxData[2] & 0xFF));
}
