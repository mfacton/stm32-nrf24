/*
 * nrf24l01p.c
 *
 *  Created on: Jul 3, 2023
 *      Author: mfact
 */

#include "nrf24l01p.h"

#include "main.h"

void nrf24l01p_create_handle(nrf24l01p_HandleTypeDef *nrf, SPI_HandleTypeDef *hspi, uint16_t spi_timeout, GPIO_TypeDef *csn_port, uint16_t csn_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin) {
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

void nrf24l01p_set_rx_pipe_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_EN_RXADDR, pipe, state);
}
nrf24l01p_state nrf24l01p_get_rx_pipe_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe) {
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

void nrf24l01p_set_rx_continuous(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state) {
	nrf24l01p_write_register_bit(nrf, NRF24L01P_REG_RF_SETUP, 7, state);
}
nrf24l01p_state nrf24l01p_get_rx_continuous(nrf24l01p_HandleTypeDef *nrf) {
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

__weak void nrf24l01p_rx_irq_callback(uint8_t *buf, uint8_t len) {
	UNUSED(buf);
	UNUSED(len);
}
