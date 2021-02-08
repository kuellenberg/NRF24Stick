/*
 * nrf24.c
 *
 *  Created on: 25.11.2020
 *      Author: André Küllenberg
 */


#include "nrf24.h"

// Peripheral libraries
#include "stm32l4xx_hal.h"

void nRF24_Init(SPI_HandleTypeDef *hspi)
{
	nrf24_hspi = hspi;
	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

uint8_t nRF24_Command(uint8_t cmd)
{
	uint8_t recv;

	nRF24_CSN_LOW;
	HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &recv, 1, HAL_MAX_DELAY);
	nRF24_CSN_HIGH;

	return recv;
}

void nRF24_WriteRegister(uint8_t reg, uint8_t data)
{
	uint8_t cmd;

	cmd = nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP);

	nRF24_CSN_LOW;
	HAL_SPI_Transmit(nrf24_hspi, &cmd, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(nrf24_hspi, &data, 1, HAL_MAX_DELAY);
	nRF24_CSN_HIGH;
}

void nRF24_WriteMBRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t cmd;
	uint8_t i;

	cmd = nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP);

	nRF24_CSN_LOW;
	HAL_SPI_Transmit(nrf24_hspi, &cmd, 1, HAL_MAX_DELAY);
	for(i = 0; i < len; i++)
		HAL_SPI_Transmit(nrf24_hspi, &buf[i], 1, HAL_MAX_DELAY);
	nRF24_CSN_HIGH;
}

uint8_t nRF24_ReadRegister(uint8_t reg)
{
	uint8_t cmd, recv;

	cmd = nRF24_CMD_R_REGISTER | (reg & nRF24_MASK_REG_MAP);

	nRF24_CSN_LOW;
	HAL_SPI_Transmit(nrf24_hspi, &cmd, 1, HAL_MAX_DELAY);
	cmd = nRF24_CMD_NOP;
	HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &recv, 1, HAL_MAX_DELAY);
	nRF24_CSN_HIGH;

	return recv;
}

void nRF24_ReadMBRegister(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t cmd;
	uint8_t i;

	cmd = nRF24_CMD_R_REGISTER | (reg & nRF24_MASK_REG_MAP);

	nRF24_CSN_LOW;
	HAL_SPI_Transmit(nrf24_hspi, &cmd, 1, HAL_MAX_DELAY);
	cmd = nRF24_CMD_NOP;
	for(i = 0; i < len; i++)
		HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &buf[i], 1, HAL_MAX_DELAY);
	nRF24_CSN_HIGH;
}

uint8_t nRF24_GetStatus(void)
{
	return nRF24_Command(nRF24_CMD_NOP);
}

void nRF24_SetRFChannel(uint8_t channel)
{
	nRF24_WriteRegister(nRF24_REG_RF_CH, channel);
}

void nRF24_SetRole(uint8_t role)
{
	uint8_t reg;

	reg = nRF24_ReadRegister(nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PRIM_RX;
	reg |= nRF24_CONFIG_PRIM_RX & role;
	nRF24_WriteRegister(nRF24_REG_CONFIG, reg);
}

void nRF24_SetPowerMode(uint8_t mode)
{
	uint8_t reg;

	reg = nRF24_ReadRegister(nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PWR_UP;
	reg |= nRF24_CONFIG_PWR_UP & mode;
	nRF24_WriteRegister(nRF24_REG_CONFIG, reg);
}

void nRF24_SetRFPower(uint8_t power)
{
	uint8_t reg;

	reg = nRF24_ReadRegister(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_RF_PWR;
	reg |= nRF24_MASK_RF_PWR & power;
	nRF24_WriteRegister(nRF24_REG_RF_SETUP, reg);
}

void nRF24_SetDatarate(uint8_t datarate)
{
	uint8_t reg;

	reg = nRF24_ReadRegister(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_DATARATE;
	reg |= nRF24_MASK_DATARATE & datarate;
	nRF24_WriteRegister(nRF24_REG_RF_SETUP, reg);
}

void nRF24_SetCRC(uint8_t crc)
{
	uint8_t reg;

	reg = nRF24_ReadRegister(nRF24_REG_CONFIG);
	reg &= ~nRF24_MASK_CRC;
	reg |= nRF24_MASK_CRC & crc;
	nRF24_WriteRegister(nRF24_REG_CONFIG, reg);
}

uint8_t nRF24_GetAddrWidth(void)
{
	return nRF24_ReadRegister(nRF24_REG_SETUP_AW) + 2;
}

void nRF24_SetAddrWidth(uint8_t aw)
{
	nRF24_WriteRegister(nRF24_REG_SETUP_AW, aw - 2);
}

void nRF24_SetAddr(uint8_t pipe, uint8_t *addr)
{
	uint8_t aw;

	switch (pipe)
	{
	case nRF24_PIPETX:
	case nRF24_PIPE0:
	case nRF24_PIPE1:
		aw = nRF24_GetAddrWidth();
		break;
	default:
		aw = 1;
	}
	nRF24_WriteMBRegister(pipe, addr, aw);
}

void nRF24_SetARD(uint8_t ard)
{
	uint8_t reg;

	reg = nRF24_ReadRegister(nRF24_REG_SETUP_RETR);
	reg &= ~nRF24_MASK_ARD;
	reg |= nRF24_MASK_ARD & ard;
	nRF24_WriteRegister(nRF24_REG_SETUP_RETR, reg);
}

void nRF24_SetARC(uint8_t arc)
{
	uint8_t reg;

	reg = nRF24_ReadRegister(nRF24_REG_SETUP_RETR);
	reg &= ~nRF24_MASK_ARC;
	reg |= nRF24_MASK_ARC & arc;
	nRF24_WriteRegister(nRF24_REG_SETUP_RETR, reg);
}

void nRF24_FlushTX(void)
{
	nRF24_Command(nRF24_CMD_FLUSH_TX);
}

void nRF24_FlushRX(void)
{
	nRF24_Command(nRF24_CMD_FLUSH_RX);
}

void nRF24_WriteTXPayload(uint8_t *payload, uint8_t len)
{
	uint8_t cmd, i;

	cmd = nRF24_CMD_W_TX_PAYLOAD;
	nRF24_CSN_LOW;
	HAL_SPI_Transmit(nrf24_hspi, &cmd, 1, HAL_MAX_DELAY);
	cmd = nRF24_CMD_NOP;
	for(i = 0; i < len; i++)
		HAL_SPI_Transmit(nrf24_hspi, &payload[i], 1, HAL_MAX_DELAY);
	nRF24_CSN_HIGH;
}

void nRF24_DisableAA(uint8_t pipe)
{
	nRF24_WriteRegister(nRF24_REG_EN_AA, 0x00);
}
