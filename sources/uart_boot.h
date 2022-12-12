/**
 ******************************************************************************
 * @author: GaoKong
 * @date:   15/09/2017
 * @brief:  Host UART for AK kernel bootloader
 ******************************************************************************
**/
#ifndef __UART_BOOT_H__
#define __UART_BOOT_H__

#include <stdint.h>

/* uart_boot_frame_t */
#define UART_BOOT_FRAME_DATA_SIZE			(256)
#define UART_BOOT_CMD_DATA_SIZE				(128)

/* uart_boot_command_t */
#define UART_BOOT_CMD_HANDSHAKE_REQ			(0x01)
#define UART_BOOT_CMD_HANDSHAKE_RES			(0x02)
#define UART_BOOT_CMD_UPDATE_REQ			(0x03)
#define UART_BOOT_CMD_UPDATE_RES			(0x04)
#define UART_BOOT_CMD_TRANSFER_FW_REQ		(0x05)
#define UART_BOOT_CMD_TRANSFER_FW_RES		(0x06)
#define UART_BOOT_CMD_CHECKSUM_FW_REQ		(0x07)
#define UART_BOOT_CMD_CHECKSUM_FW_RES		(0x08)

/* sub command */
#define UART_BOOT_SUB_CMD_1					(0x01)
#define UART_BOOT_SUB_CMD_2					(0x02)
#define UART_BOOT_SUB_CMD_3					(0x03)

#define SYS_BOOT_CMD_NONE					0x01
#define SYS_BOOT_CMD_UPDATE_REQ				0x02
#define SYS_BOOT_CMD_UPDATE_RES				0x03

#define SYS_BOOT_CONTAINER_DIRECTLY			0x01
#define SYS_BOOT_CONTAINER_EXTERNAL_FLASH	0x02
#define SYS_BOOT_CONTAINER_INTERNAL_FLASH	0x03
#define SYS_BOOT_CONTAINER_EXTERNAL_EPPROM	0x04
#define SYS_BOOT_CONTAINER_INTERNAL_EPPROM	0x05
#define SYS_BOOT_CONTAINER_SDCARD			0x06

#define SYS_BOOT_IO_DRIVER_NONE				0x01
#define SYS_BOOT_IO_DRIVER_UART				0x02
#define SYS_BOOT_IO_DRIVER_SPI				0x03

/* boot frame parser */
#define SOP_STATE		0x00
#define LEN_STATE		0x01
#define DATA_STATE		0x02
#define FCS_STATE		0x03

#define UART_BOOT_SOP_CHAR		0xEF

typedef void (*pf_uart_boot_cmd_handler)(void*);

typedef struct {
	uint8_t state;
	uint8_t sop;
	uint8_t len;
	uint8_t index;
	uint8_t data[UART_BOOT_FRAME_DATA_SIZE];
	uint8_t fcs;
	pf_uart_boot_cmd_handler uart_boot_cmd_handler;
} uart_boot_frame_t;

typedef struct {
	uint8_t cmd;
	uint8_t subcmd;
} uart_boot_cmd_t;

typedef struct {
	uart_boot_cmd_t boot_cmd;
	uint8_t len;
	uint8_t data[UART_BOOT_CMD_DATA_SIZE];
} uart_boot_data_cmd_t;

typedef struct {
	uint32_t psk;
	uint32_t bin_len;
	uint16_t checksum;
} firmware_header_t;

typedef struct {
	uint8_t type;
	uint8_t src_task_id;
	uint8_t des_task_id;
	uint8_t sig;
	uint8_t if_src_type;
	uint8_t if_des_type;
} __attribute__((packed)) ak_msg_host_res_t;

typedef struct {
	uint8_t cmd; /* none, update request, verify request ... */
	uint8_t container; /* external FLASH, EPPROM or directly via io driver... */
	uint8_t io_driver; /* SPI, UART, ... */
	uint32_t des_addr; /* start destination address */
	uint32_t src_addr; /* start source address */
	ak_msg_host_res_t ak_msg_res; /* host message response when update completed */
} firmware_boot_cmd_t;

extern void uart_boot_init(pf_uart_boot_cmd_handler);

#endif //__UART_BOOT_H__
