/*
 * nrf24l01p.c
 *
 *  Created on: Jul 3, 2023
 *      Author: mfact
 */

/*
 * SPI Speedz	= 1 Mhz
 STATUS		= 0x0e RX_DR=0 TX_DS=0 MAX_RT=0 RX_P_NO=7 TX_FULL=0
 RX_ADDR_P0-1	= 0xe7e7e7e7e7 0x3130303030
 RX_ADDR_P2-5	= 0xc3 0xc4 0xc5 0xc6
 TX_ADDR		= 0xe7e7e7e7e7
 RX_PW_P0-6	= 0x01 0x01 0x01 0x01 0x01 0x01
 EN_AA		= 0x3f
 EN_RXADDR	= 0x02
 RF_CH		= 0x4c //76
 RF_SETUP	= 0x21
 CONFIG		= 0x0f
 DYNPD/FEATURE	= 0x00 0x00
 Data Rate	= 250 KBPS
 Model		= nRF24L01+
 CRC Length	= 16 bits
 PA Power	= PA_MIN
 ARC		= 0
 */

#include "lib/nrf24l01p.h"
#include "modules/display.h"

#include "main.h"

void nrf24l01_create_handle(nrf24l01p_HandleTypeDef *nrf, SPI_HandleTypeDef *hspi, uint16_t spi_timeout, GPIO_TypeDef *csn_port, uint16_t csn_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin) {
	nrf->hspi = hspi;
	nrf->spi_timeout = spi_timeout;
	nrf->csn_port = csn_port;
	nrf->csn_pin = csn_pin;
	nrf->ce_port = ce_port;
	nrf->ce_pin = ce_pin;
}

/* Low Level Functions */
void nrf24l01p_set_csn(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	if (state == NRF_ON) {
		HAL_GPIO_WritePin(nrf->csn_port, nrf->csn_pin, GPIO_PIN_SET);
		nrf->csn_state = state;
	}else{
		HAL_GPIO_WritePin(nrf->csn_port, nrf->csn_pin, GPIO_PIN_RESET);
		nrf->csn_state = state;
	}
}
nrf24l01p_state nrf24l01p_get_csn(nrf24l01p_HandleTypeDef *nrf) {
	return nrf->csn_state;
}

void nrf24l01p_set_ce(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	if (state == NRF_ON) {
		HAL_GPIO_WritePin(nrf->ce_port, nrf->ce_pin, GPIO_PIN_SET);
		nrf->ce_state = state;
	}else{
		HAL_GPIO_WritePin(nrf->ce_port, nrf->ce_pin, GPIO_PIN_RESET);
		nrf->ce_state = state;
	}
}
nrf24l01p_state nrf24l01p_get_ce(nrf24l01p_HandleTypeDef *nrf) {
	return nrf->ce_state;
}

void nrf24l01p_command(nrf24l01p_HandleTypeDef *nrf, uint8_t command) {
	nrf24l01p_set_csn(nrf, NRF_OFF);
	HAL_SPI_TransmitReceive(nrf->hspi, &command, &nrf->status, 1, nrf->spi_timeout);
	nrf24l01p_set_csn(nrf, NRF_ON);
}

void nrf24l01p_read_command_buffer(nrf24l01p_HandleTypeDef *nrf, uint8_t command, uint8_t *buf, uint8_t len) {
	nrf24l01p_set_csn(nrf, NRF_OFF);
	HAL_SPI_TransmitReceive(nrf->hspi, &command, &nrf->status, 1, nrf->spi_timeout);
	HAL_SPI_Receive(nrf->hspi, buf, len, nrf->spi_timeout);
	nrf24l01p_set_csn(nrf, NRF_ON);
}

void nrf24l01p_write_command_buffer(nrf24l01p_HandleTypeDef *nrf, uint8_t command, uint8_t *buf, uint8_t len) {
	nrf24l01p_set_csn(nrf, NRF_OFF);
	HAL_SPI_TransmitReceive(nrf->hspi, &command, &nrf->status, 1, nrf->spi_timeout);
	HAL_SPI_Transmit(nrf->hspi, buf, len, nrf->spi_timeout);
	nrf24l01p_set_csn(nrf, NRF_ON);
}

nrf24l01p_state nrf24l01p_read_register_bit(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t bit) {
	return (nrf24l01p_read_register(nrf, reg) >> bit) & 1;
}
void nrf24l01p_write_register_bit(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t bit, nrf24l01p_state state) {
	uint8_t value = nrf24l01p_read_register(nrf, reg);
	if (state == NRF_ON) {
		value |= (1 << bit);
	}else{
		value &= ~(1 << bit);
	}
	nrf24l01p_write_register(nrf, reg, value);
}

/* Commands */
uint8_t nrf24l01p_read_register(nrf24l01p_HandleTypeDef *nrf, uint8_t reg) {
	uint8_t value;

	nrf24l01p_read_command_buffer(nrf, NRF24L01P_CMD_R_REGISTER | reg, &value, 1);

	return value;
}
void nrf24l01p_read_register_buffer(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t *buf, uint8_t len) {
	nrf24l01p_read_command_buffer(nrf, NRF24L01P_CMD_R_REGISTER | reg, buf, len);
}

void nrf24l01p_write_register(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t val) {
	nrf24l01p_write_command_buffer(nrf, NRF24L01P_CMD_W_REGISTER | reg, &val, 1);
}
void nrf24l01p_write_register_buffer(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t *buf, uint8_t len) {
	nrf24l01p_write_command_buffer(nrf, NRF24L01P_CMD_W_REGISTER | reg, buf, len);
}

uint8_t nrf24l01p_read_rx_fifo(nrf24l01p_HandleTypeDef *nrf, uint8_t *buf) {
	uint8_t len_pl = nrf24l01p_read_rx_pl_width(nrf);
	nrf24l01p_read_command_buffer(nrf, NRF24L01P_CMD_R_RX_PAYLOAD, buf, len_pl);
	return len_pl;
}
void nrf24l01p_write_tx_fifo(nrf24l01p_HandleTypeDef *nrf, uint8_t *buf, uint8_t len) {
	nrf24l01p_write_command_buffer(nrf, NRF24L01P_CMD_W_TX_PAYLOAD, buf, len);
}

void nrf24l01p_flush_tx(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_command(nrf, NRF24L01P_CMD_FLUSH_TX);
}
void nrf24l01p_flush_rx(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_command(nrf, NRF24L01P_CMD_FLUSH_RX);
}

void nrf24l01p_reuse_tx_pl(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_command(nrf, NRF24L01P_CMD_REUSE_TX_PL);
}

uint8_t nrf24l01p_read_rx_pl_width(nrf24l01p_HandleTypeDef *nrf) {
	uint8_t value;
	nrf24l01p_read_command_buffer(nrf, NRF24L01P_CMD_R_RX_PL_WID, &value, 1);
	return value;
}

void nrf24l01p_write_ack_pl(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, uint8_t *buf, uint8_t len) {
	nrf24l01p_write_command_buffer(nrf, NRF24L01P_CMD_W_ACK_PAYLOAD + pipe, buf, len);
}

void nrf24l01p_write_noack_pl(nrf24l01p_HandleTypeDef *nrf, uint8_t *buf, uint8_t len) {
	nrf24l01p_write_command_buffer(nrf, NRF24L01P_CMD_W_TX_PAYLOAD_NOACK, buf, len);
}

uint8_t nrf24l01p_get_status(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_command(nrf, NRF24L01P_CMD_NOP);
	return nrf->status;
}

/* Read Write Registers*/
void nrf24l01p_set_rx_irq(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_CONFIG, 6, 1-state);
}
nrf24l01p_state nrf24l01p_get_rx_irq(nrf24l01p_HandleTypeDef *nrf) {
	return 1-nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_CONFIG, 6);
}

void nrf24l01p_set_tx_irq(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_CONFIG, 5, 1-state);
}
nrf24l01p_state nrf24l01p_get_tx_irq(nrf24l01p_HandleTypeDef *nrf) {
	return 1-nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_CONFIG, 5);
}

void nrf24l01p_set_rt_irq(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_CONFIG, 4, 1-state);
}
nrf24l01p_state nrf24l01p_get_rt_irq(nrf24l01p_HandleTypeDef *nrf) {
	return 1-nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_CONFIG, 4);
}

void nrf24l01p_set_crc_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_CONFIG, 3, state);
}
nrf24l01p_state nrf24l01p_get_crc_en(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_CONFIG, 3);
}

void nrf24l01p_set_crc_bits(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_crc_type crc) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_CONFIG, 2, crc);
}
nrf24l01p_crc_type nrf24l01p_get_crc_bits(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_CONFIG, 2);
}

void nrf24l01p_set_power(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_CONFIG, 1, state);
}
nrf24l01p_state nrf24l01p_get_power(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_CONFIG, 1);
}

void nrf24l01p_set_mode(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_mode mode) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_CONFIG, 0, mode);
}
nrf24l01p_mode nrf24l01p_get_mode(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_CONFIG, 0);
}

void nrf24l01p_set_auto_ack_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_EN_AA, pipe, state);
}
nrf24l01p_state nrf24l01p_get_auto_ack_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_EN_AA, pipe);
}

void nrf24l01p_set_pipe_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_EN_RXADDR, pipe, state);
}
nrf24l01p_state nrf24l01p_get_pipe_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_EN_RXADDR, pipe);
}

void nrf24l01p_set_address_width(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_address_width width) {
	nrf24l01p_write_register(nrf, NRF24L01P_REG_SETUP_AW, width);
}
nrf24l01p_address_width nrf24l01p_get_address_width(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register(nrf, NRF24L01P_REG_SETUP_AW);
}

void nrf24l01p_set_rt_delay(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_rt_delay delay) {
	uint8_t value = nrf24l01p_read_register(nrf, NRF24L01P_REG_SETUP_RETR);
	value &= 0x0F;
	value |= (delay << 4);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_SETUP_RETR, value);
}
nrf24l01p_rt_delay nrf24l01p_get_rt_delay(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register(nrf, NRF24L01P_REG_SETUP_RETR) >> 4;
}

void nrf24l01p_set_rt_count(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_rt_count count) {
	uint8_t value = nrf24l01p_read_register(nrf, NRF24L01P_REG_SETUP_RETR);
	value &= 0xF0;
	value |= count;
	nrf24l01p_write_register(nrf, NRF24L01P_REG_SETUP_RETR, value);
}
nrf24l01p_rt_count nrf24l01p_get_rt_count(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register(nrf, NRF24L01P_REG_SETUP_RETR) & 0x0F;
}

void nrf24l01p_set_frequency(nrf24l01p_HandleTypeDef *nrf, uint16_t frequency) {
	uint8_t value = frequency - 2400;
	if (value > 0x7F) {
		value = 0x7F;
	}
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RF_CH, value);
}
uint16_t nrf24l01p_get_frequency(nrf24l01p_HandleTypeDef *nrf) {
	return 2400u + nrf24l01p_read_register(nrf, NRF24L01P_REG_RF_CH);
}

void nrf24l01p_set_rx_cont(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_RF_SETUP, 7, state);
}
nrf24l01p_state nrf24l01p_get_rx_cont(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register(nrf, NRF24L01P_REG_RF_SETUP);
}

void nrf24l01p_set_data_rate(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_data_rate rate) {
	uint8_t value = nrf24l01p_read_register(nrf, NRF24L01P_REG_RF_SETUP);
	if (rate == NRF_250KBPS) {
		value |= (1 << 5);
		value &= ~(1 << 3);
	}else {
		value &= ~(1 << 5);
		value |= (rate << 3);
	}
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RF_SETUP, value);
}
nrf24l01p_data_rate nrf24l01p_get_data_rate(nrf24l01p_HandleTypeDef *nrf) {
	uint8_t value = nrf24l01p_read_register(nrf, NRF24L01P_REG_RF_SETUP);
	if ((value >> 5) & 1) {
		return NRF_250KBPS;
	}else{
		return (value >> 3) & 1;
	}
}

void nrf24l01p_set_output_power(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_output_power power) {
	uint8_t value = nrf24l01p_read_register(nrf, NRF24L01P_REG_RF_SETUP);
	value &= 0xF8;
	value |= (power << 1);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RF_SETUP, value);
}
nrf24l01p_output_power nrf24l01p_get_output_power(nrf24l01p_HandleTypeDef *nrf) {
	return ((nrf24l01p_read_register(nrf, NRF24L01P_REG_RF_SETUP) & 0x07) >> 1) & 0x03;
}

void nrf24l01p_set_rx_dr(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_STATUS, 6, NRF_ON);
}
uint8_t nrf24l01p_get_rx_dr(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_STATUS, 6);
}

void nrf24l01p_set_tx_ds(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_STATUS, 5, NRF_ON);
}
uint8_t nrf24l01p_get_tx_ds(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_STATUS, 5);
}

void nrf24l01p_set_max_rt(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_STATUS, 4, NRF_ON);
}
uint8_t nrf24l01p_get_max_rt(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_STATUS, 4);
}

nrf24l01p_pipe nrf24l01p_get_pipe_available(nrf24l01p_HandleTypeDef *nrf) {
	return (nrf24l01p_read_register(nrf, NRF24L01P_REG_STATUS) >> 1) & 0x07;
}

uint8_t nrf24l01p_get_plos_cnt(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register(nrf, NRF24L01P_REG_OBSERVE_TX) >> 4;
}

uint8_t nrf24l01p_get_rt_cnt(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register(nrf, NRF24L01P_REG_OBSERVE_TX) & 0x0F;
}

uint8_t nrf24l01p_get_detect_power(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register(nrf, NRF24L01P_REG_OBSERVE_TX);
}

void nrf24l01p_set_rx_addr(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, uint8_t *buff, uint8_t len) {
	nrf24l01p_write_register_buffer(nrf, NRF24L01P_REG_RX_ADDR_P0+pipe, buff, len);
}
void nrf24l01p_get_rx_addr(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, uint8_t *buff, uint8_t len) {
	nrf24l01p_read_register_buffer(nrf, NRF24L01P_REG_RX_ADDR_P0+pipe, buff, len);
}

void nrf24l01p_set_tx_addr(nrf24l01p_HandleTypeDef *nrf, uint8_t *buff) {
	nrf24l01p_write_register_buffer(nrf, NRF24L01P_REG_TX_ADDR, buff, 5);
}
void nrf24l01p_get_tx_addr(nrf24l01p_HandleTypeDef *nrf, uint8_t *buff) {
	nrf24l01p_read_register_buffer(nrf, NRF24L01P_REG_TX_ADDR, buff, 5);
}

void nrf24l01p_set_payload_size(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, uint8_t size) {
	if (size > 32) {
		size = 32;
	}
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_PW_P0+pipe, size);
}

uint8_t nrf24l01p_get_payload_size(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe) {
	return nrf24l01p_read_register(nrf, NRF24L01P_REG_RX_PW_P0+pipe);
}

uint8_t nrf24l01p_get_tx_reuse(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_FIFO_STATUS, 6);
}

nrf24l01p_fifo_status nrf24l01p_get_tx_fifo(nrf24l01p_HandleTypeDef *nrf) {
	if (nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_FIFO_STATUS, 4)) {
		return NRF_FIFO_EMPTY;
	}else if (nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_FIFO_STATUS, 5)) {
		return NRF_FIFO_FULL;
	}else{
		return NRF_FIFO_AVAILABLE;
	}
}
nrf24l01p_fifo_status nrf24l01p_get_rx_fifo(nrf24l01p_HandleTypeDef *nrf) {
	if (nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_FIFO_STATUS, 0)) {
		return NRF_FIFO_EMPTY;
	}else if (nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_FIFO_STATUS, 1)) {
		return NRF_FIFO_FULL;
	}else{
		return NRF_FIFO_AVAILABLE;
	}
}

void nrf24l01p_set_dpl(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_DYNPD, pipe, state);
}
nrf24l01p_state nrf24l01p_get_dpl(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_DYNPD, pipe);
}

void nrf24l01p_set_dpl_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_FEATURE, 2, state);
}
nrf24l01p_state nrf24l01p_get_dpl_en(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_FEATURE, 2);
}

void nrf24l01p_set_ack_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_FEATURE, 1, state);
}
nrf24l01p_state nrf24l01p_get_ack_en(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_FEATURE, 1);
}

void nrf24l01p_set_dyn_ack_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_FEATURE, 0, state);
}
nrf24l01p_state nrf24l01p_get_dyn_ack_en(nrf24l01p_HandleTypeDef *nrf) {
	return nrf24l01p_read_register_bit(nrf, NRF24L01P_REG_FEATURE, 0);
}

/* Main Functions */
void nrf24l01p_reset(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_set_power(nrf, NRF_OFF);
	nrf24l01p_set_ce(nrf, NRF_OFF);

	nrf24l01p_write_register(nrf, NRF24L01P_REG_CONFIG, 0x08);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_EN_AA, 0x3F);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_EN_RXADDR, 0x03);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_SETUP_AW, 0x03);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_SETUP_RETR, 0x03);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RF_CH, 0x02);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RF_SETUP, 0x0E);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_STATUS, 0x0E);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_OBSERVE_TX, 0x00);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RPD, 0x00);

	uint8_t buf0[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	nrf24l01p_write_register_buffer(nrf, NRF24L01P_REG_RX_ADDR_P0, buf0, 5);
	uint8_t buf1[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	nrf24l01p_write_register_buffer(nrf, NRF24L01P_REG_RX_ADDR_P1, buf1, 5);
	uint8_t address = 0xC3;
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_ADDR_P2, address);
	address++;
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_ADDR_P3, address);
	address++;
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_ADDR_P4, address);
	address++;
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_ADDR_P5, address);
	nrf24l01p_write_register_buffer(nrf, NRF24L01P_REG_TX_ADDR, buf0, 5);

	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_PW_P0, 0x00);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_PW_P1, 0x00);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_PW_P2, 0x00);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_PW_P3, 0x00);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_PW_P4, 0x00);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_RX_PW_P5, 0x00);

	nrf24l01p_write_register(nrf, NRF24L01P_REG_FIFO_STATUS, 0x11);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_DYNPD, 0x00);
	nrf24l01p_write_register(nrf, NRF24L01P_REG_FEATURE, 0x00);
}

//general outline, but recommended to manually initialize
void nrf24l01p_rx_init(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_set_mode(nrf, NRF_RECIEVE);
	nrf24l01p_set_power(nrf, NRF_ON);

	nrf24l01p_set_tx_irq(nrf, NRF_OFF);
	nrf24l01p_set_rt_irq(nrf, NRF_OFF);

	nrf24l01p_set_rx_irq(nrf, NRF_ON);
	nrf24l01p_flush_rx(nrf);
	nrf24l01p_set_rx_dr(nrf);

	nrf24l01p_set_ce(nrf, NRF_ON);
}

//general outline, but recommended to manually initialize
void nrf24l01p_tx_init(nrf24l01p_HandleTypeDef *nrf) {
	nrf24l01p_set_mode(nrf, NRF_TRANSMIT);
	nrf24l01p_set_power(nrf, NRF_ON);

	nrf24l01p_set_rx_irq(nrf, NRF_OFF);

	nrf24l01p_set_tx_irq(nrf, NRF_ON);
	nrf24l01p_set_rt_irq(nrf, NRF_ON);
	nrf24l01p_flush_tx(nrf);
	nrf24l01p_set_tx_ds(nrf);

	nrf24l01p_set_ce(nrf, NRF_ON);
}

void nrf24l01p_rx_receive(nrf24l01p_HandleTypeDef *nrf, uint8_t* buf) {

}
void nrf24l01p_tx_transmit(nrf24l01p_HandleTypeDef *nrf, uint8_t* buf) {

}

void nrf24l01p_irq_handler(nrf24l01p_HandleTypeDef *nrf) {

}

__weak void nrf24l01p_irq_callback() {

}

/*
static void cs_high() {
	HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER,
			GPIO_PIN_SET);
}

static void cs_low() {
	HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER,
			GPIO_PIN_RESET);
}

static void ce_high() {
	HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER,
			GPIO_PIN_SET);
}

static void ce_low() {
	HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER,
			GPIO_PIN_RESET);
}

static uint8_t read_register(uint8_t reg) {
	uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
	uint8_t status;
	uint8_t read_val;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	HAL_SPI_Receive(NRF24L01P_SPI, &read_val, 1, 2000);
	cs_high();

	return read_val;
}

static void read_register_buffer(uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	HAL_SPI_Receive(NRF24L01P_SPI, buf, len, 2000);
	cs_high();
}

static uint8_t write_register(uint8_t reg, uint8_t value) {
	uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
	uint8_t status;
	uint8_t write_val = value;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	HAL_SPI_Transmit(NRF24L01P_SPI, &write_val, 1, 2000);
	cs_high();

	return write_val;
}

static void write_register_buffer(uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	HAL_SPI_Transmit(NRF24L01P_SPI, buf, len, 2000);
	cs_high();
}

// nRF24L01+ Main Functions
void nrf24l01p_rx_init(channel MHz, air_data_rate bps) {
//	nrf24l01p_set_address_widths(5);
	nrf24l01p_reset();

	nrf24l01p_prx_mode();
	nrf24l01p_power_up();

//	nrf24l01p_rx_set_payload_widths(NRF24L01P_PAYLOAD_LENGTH);

	nrf24l01p_set_rf_air_data_rate(_250kbps);

	nrf24l01p_set_crc_length(2);

//	nrf24l01p_auto_retransmit_count(3);
//	nrf24l01p_auto_retransmit_delay(250);

	ce_high();

	HAL_Delay(1000);

	uint8_t val = read_register(NRF24L01P_REG_CONFIG);
	Display_PrintValuePosition(2, 2, val);
}

void nrf24l01p_tx_init(channel MHz, air_data_rate bps) {
	nrf24l01p_reset();

	nrf24l01p_ptx_mode();
	nrf24l01p_power_up();

	nrf24l01p_set_rf_air_data_rate(_250kbps);

	nrf24l01p_set_crc_length(2);
	nrf24l01p_set_address_widths(5);

	nrf24l01p_auto_retransmit_count(3);
	nrf24l01p_auto_retransmit_delay(250);

	ce_high();
}

void nrf24l01p_rx_receive(uint8_t *rx_payload) {
	nrf24l01p_read_rx_fifo(rx_payload);
	nrf24l01p_clear_rx_dr();
}

void nrf24l01p_tx_transmit(uint8_t *tx_payload) {
	nrf24l01p_write_tx_fifo(tx_payload);
}

void nrf24l01p_tx_irq() {
	uint8_t tx_ds = nrf24l01p_get_status();
	tx_ds &= 0x20;

	if (tx_ds) {
		// TX_DS
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		nrf24l01p_clear_tx_ds();
	}

	else {
		// MAX_RT
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
		nrf24l01p_clear_max_rt();
	}
}

// nRF24L01+ Sub Functions
void nrf24l01p_reset() {
	// Reset pins
	cs_high();
	ce_low();

	// Reset registers
	write_register(NRF24L01P_REG_CONFIG, 0x0c);
	write_register(NRF24L01P_REG_EN_AA, 0x3F);
	write_register(NRF24L01P_REG_EN_RXADDR, 0x02); //en pipe 1
	write_register(NRF24L01P_REG_SETUP_AW, 0x03);
	write_register(NRF24L01P_REG_SETUP_RETR, 0x03);
	write_register(NRF24L01P_REG_RF_CH, 0x4c);
	write_register(NRF24L01P_REG_RF_SETUP, 0x06);
	write_register(NRF24L01P_REG_STATUS, 0x7E);

	//pyload byte sizes
	write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
	write_register(NRF24L01P_REG_RX_PW_P1, 0x01); //1 byte
	write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
	write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
	write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
	write_register(NRF24L01P_REG_RX_PW_P5, 0x00);

	write_register(NRF24L01P_REG_FIFO_STATUS, 0x11);
	write_register(NRF24L01P_REG_DYNPD, 0x00);
	write_register(NRF24L01P_REG_FEATURE, 0x00);

	uint8_t address[5] = "00001";
	write_register_buffer(NRF24L01P_REG_RX_ADDR_P1, address, 5);

	// Reset FIFO
	nrf24l01p_flush_rx_fifo();
	nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_prx_mode() {
	uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
	new_config |= 1;

	write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx_mode() {
	uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
	new_config &= 0xFE;

	write_register(NRF24L01P_REG_CONFIG, new_config);
}

uint8_t nrf24l01p_rx_available() {
//	uint8_t reg = read_register(NRF24L01P_REG_STATUS);
//	write_register(NRF24L01P_REG_STATUS, 0x27);
//	uint8_t reg = read_register(NRF24L01P_REG_STATUS);
//	Display_PrintValuePosition(2, 2, reg);
//	uint8_t stat = (~reg>>6) & 1;
//	if (stat == 1) {
//		Display_PrintValuePosition(2, 2, reg);
//	}else{
//		Display_PrintValuePosition(2, 2, 69);
//	}
	return 0;
}

uint8_t nrf24l01p_read_rx_fifo(uint8_t *rx_payload) {
	uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	HAL_SPI_Receive(NRF24L01P_SPI, rx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
	cs_high();

	return status;
}

uint8_t nrf24l01p_write_tx_fifo(uint8_t *tx_payload) {
	uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	HAL_SPI_Transmit(NRF24L01P_SPI, tx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
	cs_high();

	return status;
}

void nrf24l01p_flush_rx_fifo() {
	uint8_t command = NRF24L01P_CMD_FLUSH_RX;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	cs_high();
}

void nrf24l01p_flush_tx_fifo() {
	uint8_t command = NRF24L01P_CMD_FLUSH_TX;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	cs_high();
}

uint8_t nrf24l01p_get_status() {
	uint8_t command = NRF24L01P_CMD_NOP;
	uint8_t status;

	cs_low();
	HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
	cs_high();

	return status;
}

uint8_t nrf24l01p_get_fifo_status() {
	return read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_set_payload_widths(widths bytes) {
	write_register(NRF24L01P_REG_RX_PW_P1, bytes);
}

void nrf24l01p_clear_rx_dr() {
	uint8_t new_status = nrf24l01p_get_status();
	new_status |= 0x40;

	write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_tx_ds() {
	uint8_t new_status = nrf24l01p_get_status();
	new_status |= 0x20;

	write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_max_rt() {
	uint8_t new_status = nrf24l01p_get_status();
	new_status |= 0x10;

	write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_power_up() {
	uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
	new_config |= 1 << 1;

	write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down() {
	uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
	new_config &= 0xFD;

	write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(length bytes) {
	uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);

	switch (bytes) {
	// CRCO bit in CONFIG resiger set 0
	case 1:
		new_config &= 0xFB;
		break;
		// CRCO bit in CONFIG resiger set 1
	case 2:
		new_config |= 1 << 2;
		break;
	}

	write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(widths bytes) {
	write_register(NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_auto_retransmit_count(count cnt) {
	uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

	// Reset ARC register 0
	new_setup_retr |= 0xF0;
	new_setup_retr |= cnt;
	write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_delay(delay us) {
	uint8_t new_setup_retr = read_register(NRF24L01P_REG_SETUP_RETR);

	// Reset ARD register 0
	new_setup_retr |= 0x0F;
	new_setup_retr |= ((us / 250) - 1) << 4;
	write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_rf_channel(channel MHz) {
	uint16_t new_rf_ch = MHz - 2400;
	write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(output_power dBm) {
	uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
	new_rf_setup |= (dBm << 1);

	write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(air_data_rate bps) {
	// Set value to 0
	uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;

	switch (bps) {
	case _1Mbps:
		break;
	case _2Mbps:
		new_rf_setup |= 1 << 3;
		break;
	case _250kbps:
		new_rf_setup |= 1 << 5;
		break;
	}
	write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}
*/
