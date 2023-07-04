/*
 * nrf24l01.h
 *
 *  Created on: Jul 3, 2023
 *      Author: mfact
 */

#pragma once

#include "main.h"

/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER                  0b00000000
#define NRF24L01P_CMD_W_REGISTER                  0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD                0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD                0b10100000
#define NRF24L01P_CMD_FLUSH_TX                    0b11100001
#define NRF24L01P_CMD_FLUSH_RX                    0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL                 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID                 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD               0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK          0b10110000
#define NRF24L01P_CMD_NOP                         0b11111111

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG            0x00
#define NRF24L01P_REG_EN_AA             0x01
#define NRF24L01P_REG_EN_RXADDR         0x02
#define NRF24L01P_REG_SETUP_AW          0x03
#define NRF24L01P_REG_SETUP_RETR        0x04
#define NRF24L01P_REG_RF_CH             0x05
#define NRF24L01P_REG_RF_SETUP          0x06
#define NRF24L01P_REG_STATUS            0x07
#define NRF24L01P_REG_OBSERVE_TX        0x08 // Read-Only
#define NRF24L01P_REG_RPD               0x09 // Read-Only
#define NRF24L01P_REG_RX_ADDR_P0        0x0A
#define NRF24L01P_REG_RX_ADDR_P1        0x0B
#define NRF24L01P_REG_RX_ADDR_P2        0x0C
#define NRF24L01P_REG_RX_ADDR_P3        0x0D
#define NRF24L01P_REG_RX_ADDR_P4        0x0E
#define NRF24L01P_REG_RX_ADDR_P5        0x0F
#define NRF24L01P_REG_TX_ADDR           0x10
#define NRF24L01P_REG_RX_PW_P0          0x11
#define NRF24L01P_REG_RX_PW_P1          0x12
#define NRF24L01P_REG_RX_PW_P2          0x13
#define NRF24L01P_REG_RX_PW_P3          0x14
#define NRF24L01P_REG_RX_PW_P4          0x15
#define NRF24L01P_REG_RX_PW_P5          0x16
#define NRF24L01P_REG_FIFO_STATUS       0x17 //Read-Only
#define NRF24L01P_REG_DYNPD             0x1C
#define NRF24L01P_REG_FEATURE           0x1D

typedef enum {
	NRF_ON = 1u,
	NRF_OFF = 0u
}nrf24l01p_state;

typedef enum {
	NRF_TRANSMIT = 0u,
	NRF_RECIEVE = 1u
} nrf24l01p_mode;

typedef enum
{
    NRF_250KBPS = 2u,
    NRF_1MBPS = 0u,
    NRF_2MBPS = 1u
} nrf24l01p_data_rate;

//N -> negative
typedef enum
{
    NRF_N18DBM = 0u,
    NRF_N12DBM = 1u,
    NRF_N6DBM = 2u,
    NRF_0DBM = 3u
} nrf24l01p_output_power;

/* CRC bit sizes */
typedef enum
{
	NRF_CRC_8 = 0u,
	NRF_CRC_16 = 1u
} nrf24l01p_crc_type;

typedef enum {
	NRF_PIPE_0 = 0u,
	NRF_PIPE_1 = 1u,
	NRF_PIPE_2 = 2u,
	NRF_PIPE_3 = 3u,
	NRF_PIPE_4 = 4u,
	NRF_PIPE_5 = 5u,
	NRF_NOT_USED = 6u,
	NRF_NOT_AVAILABLE = 7u,
}nrf24l01p_pipe;

typedef enum {
	NRF_WIDTH_3 = 1u,
	NRF_WIDTH_4 = 2u,
	NRF_WIDTH_5 = 3u
}nrf24l01p_address_width;

//US -> microseconds
typedef enum {
	NRF_250US = 0u,
	NRF_500US = 1u,
	NRF_750US = 2u,
	NRF_1000US = 3u,
	NRF_1250US = 4u,
	NRF_1500US = 5u,
	NRF_1750US = 6u,
	NRF_2000US = 7u,
	NRF_2250US = 8u,
	NRF_2500US = 9u,
	NRF_2750US = 10u,
	NRF_3000US = 11u,
	NRF_3250US = 12u,
	NRF_3500US = 13u,
	NRF_3750US = 14u,
	NRF_4000US = 15u,
}nrf24l01p_rt_delay;

typedef enum {
	NRF_RT_OFF = 0u,
	NRF_RT_1 = 1u,
	NRF_RT_2 = 2u,
	NRF_RT_3 = 3u,
	NRF_RT_4 = 4u,
	NRF_RT_5 = 5u,
	NRF_RT_6 = 6u,
	NRF_RT_7 = 7u,
	NRF_RT_8 = 8u,
	NRF_RT_9 = 9u,
	NRF_RT_10 = 10u,
	NRF_RT_11 = 11u,
	NRF_RT_12 = 12u,
	NRF_RT_13 = 13u,
	NRF_RT_14 = 14u,
	NRF_RT_15 = 15u,
}nrf24l01p_rt_count;

typedef enum {
	NRF_FIFO_EMPTY = 0u,
	NRF_FIFO_AVAILABLE,
	NRF_FIFO_FULL
}nrf24l01p_fifo_status;

typedef struct {
	SPI_HandleTypeDef *hspi;
	uint16_t spi_timeout;

	GPIO_TypeDef *csn_port;
	uint16_t csn_pin;
	nrf24l01p_state csn_state;

	GPIO_TypeDef *ce_port;
	uint16_t ce_pin;
	nrf24l01p_state ce_state;

	uint8_t status;
}nrf24l01p_HandleTypeDef;

void nrf24l01_create_handle(nrf24l01p_HandleTypeDef *nrf, SPI_HandleTypeDef *hspi, uint16_t spi_timeout, GPIO_TypeDef *csn_port, uint16_t csn_pin, GPIO_TypeDef *ce_port, uint16_t ce_pin);

/* Low Level Functions */
void nrf24l01p_set_csn(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_csn(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_ce(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_ce(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_command(nrf24l01p_HandleTypeDef *nrf, uint8_t command);
void nrf24l01p_read_command_buffer(nrf24l01p_HandleTypeDef *nrf, uint8_t command, uint8_t *buf, uint8_t len);
void nrf24l01p_write_command_buffer(nrf24l01p_HandleTypeDef *nrf, uint8_t command, uint8_t *buf, uint8_t len);

nrf24l01p_state nrf24l01p_read_register_bit(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t bit);
void nrf24l01p_write_register_bit(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t bit, nrf24l01p_state state);


/* Commands */
uint8_t nrf24l01p_read_register(nrf24l01p_HandleTypeDef *nrf, uint8_t reg);
void nrf24l01p_read_register_buffer(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t *buf, uint8_t len);

void nrf24l01p_write_register(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t val);
void nrf24l01p_write_register_buffer(nrf24l01p_HandleTypeDef *nrf, uint8_t reg, uint8_t *buf, uint8_t len);

//returns payload (buf) length
uint8_t nrf24l01p_read_rx_fifo(nrf24l01p_HandleTypeDef *nrf, uint8_t *buf);
void nrf24l01p_write_tx_fifo(nrf24l01p_HandleTypeDef *nrf, uint8_t *buf, uint8_t len);

void nrf24l01p_flush_tx(nrf24l01p_HandleTypeDef *nrf);
void nrf24l01p_flush_rx(nrf24l01p_HandleTypeDef *nrf);

//pl -> payload
void nrf24l01p_reuse_tx_pl(nrf24l01p_HandleTypeDef *nrf);

uint8_t nrf24l01p_read_rx_pl_width(nrf24l01p_HandleTypeDef *nrf);

//write payload to be transmitted with ack packet
void nrf24l01p_write_ack_pl(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, uint8_t *buf, uint8_t len);

void nrf24l01p_write_noack_pl(nrf24l01p_HandleTypeDef *nrf, uint8_t *buf, uint8_t len);

uint8_t nrfl0124p_get_status(nrf24l01p_HandleTypeDef *nrf);

/* Read Write Registers*/
void nrf24l01p_set_rx_irq(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_rx_irq(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_tx_irq(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_tx_irq(nrf24l01p_HandleTypeDef *nrf);

//rt -> retransmits
void nrf24l01p_set_rt_irq(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_rt_irq(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_crc_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_crc_en(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_crc_bits(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_crc_type crc);
nrf24l01p_crc_type nrf24l01p_get_crc_bits(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_power(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_power(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_mode(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_mode mode);
nrf24l01p_mode nrf24l01p_get_mode(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_auto_ack_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_auto_ack_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe);

void nrf24l01p_set_pipe_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_pipe_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe);

void nrf24l01p_set_address_width(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_address_width width);
nrf24l01p_address_width nrf24l01p_get_address_width(nrf24l01p_HandleTypeDef *nrf);

//rt -> retransmits
void nrf24l01p_set_rt_delay(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_rt_delay delay);
nrf24l01p_rt_delay nrf24l01p_get_rt_delay(nrf24l01p_HandleTypeDef *nrf);

//count 4 bits, 0 -> off
void nrf24l01p_set_rt_count(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_rt_count count);
nrf24l01p_rt_count nrf24l01p_get_rt_count(nrf24l01p_HandleTypeDef *nrf);

//2400 - 2527 Mhz
void nrf24l01p_set_frequency(nrf24l01p_HandleTypeDef *nrf, uint16_t frequency);
uint16_t nrf24l01p_get_frequency(nrf24l01p_HandleTypeDef *nrf);

//continuous carrier transmit
void nrf24l01p_set_rx_cont(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_rx_cont(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_data_rate(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_data_rate rate);
nrf24l01p_data_rate nrf24l01p_get_data_rate(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_output_power(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_output_power power);
nrf24l01p_output_power nrf24l01p_get_output_power(nrf24l01p_HandleTypeDef *nrf);

//dr -> data ready
void nrf24l01p_set_rx_dr(nrf24l01p_HandleTypeDef *nrf);
uint8_t nrf24l01p_get_rx_dr(nrf24l01p_HandleTypeDef *nrf);

//ds -> data sent
void nrf24l01p_set_tx_ds(nrf24l01p_HandleTypeDef *nrf);
uint8_t nrf24l01p_get_tx_ds(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_set_max_rt(nrf24l01p_HandleTypeDef *nrf);
uint8_t nrf24l01p_get_max_rt(nrf24l01p_HandleTypeDef *nrf);

nrf24l01p_pipe nrf24l01p_get_pipe_available(nrf24l01p_HandleTypeDef *nrf);

//15 max
uint8_t nrf24l01p_get_plos_cnt(nrf24l01p_HandleTypeDef *nrf);

//15 max
uint8_t nrf24l01p_get_rt_cnt(nrf24l01p_HandleTypeDef *nrf);

//(power > -64 dBm) ? 1 : 0
uint8_t nrf24l01p_get_detect_power(nrf24l01p_HandleTypeDef *nrf);

//pipe 0-1 -> 5 bytes
//pipe 2-5 -> 1 byte
void nrf24l01p_set_rx_addr(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, uint8_t *buff, uint8_t len);
void nrf24l01p_get_rx_addr(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, uint8_t *buff, uint8_t len);

//5 bytes
void nrf24l01p_set_tx_addr(nrf24l01p_HandleTypeDef *nrf, uint8_t *buff);
void nrf24l01p_get_tx_addr(nrf24l01p_HandleTypeDef *nrf, uint8_t *buff);

//0 -> pipe not used
//1-32 -> byte size
void nrf24l01p_set_payload_size(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, uint8_t size);
uint8_t nrf24l01p_get_payload_size(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe);

uint8_t nrf24l01p_get_tx_reuse(nrf24l01p_HandleTypeDef *nrf);

nrf24l01p_fifo_status nrf24l01p_get_tx_fifo(nrf24l01p_HandleTypeDef *nrf);
nrf24l01p_fifo_status nrf24l01p_get_rx_fifo(nrf24l01p_HandleTypeDef *nrf);

//dpl -> dynamic payload length
void nrf24l01p_set_dpl(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_dpl(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_pipe pipe);

void nrf24l01p_set_dpl_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_dpl_en(nrf24l01p_HandleTypeDef *nrf);

//ack -> payload with awknowledge
void nrf24l01p_set_ack_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_ack_en(nrf24l01p_HandleTypeDef *nrf);

//enable W_TX_PAYLOAD_NOACK command
void nrf24l01p_set_dyn_ack_en(nrf24l01p_HandleTypeDef *nrf, nrf24l01p_state state);
nrf24l01p_state nrf24l01p_get_dyn_ack_en(nrf24l01p_HandleTypeDef *nrf);

/* Main Functions */
void nrf24l01p_reset(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_rx_init(nrf24l01p_HandleTypeDef *nrf);
void nrf24l01p_tx_init(nrf24l01p_HandleTypeDef *nrf);

void nrf24l01p_rx_receive(nrf24l01p_HandleTypeDef *nrf, uint8_t* buf);
void nrf24l01p_tx_transmit(nrf24l01p_HandleTypeDef *nrf, uint8_t* buf);

//should be called in the interrupt callback
void nrf24l01p_irq_handler(nrf24l01p_HandleTypeDef *nrf);

//redefine this with what you want to happen when data is recieved
__weak void nrf24l01p_rx_irq_callback(uint8_t *buf, uint8_t len);
