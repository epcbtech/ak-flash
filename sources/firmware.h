#ifndef __FIRMWARE_H__
#define __FIRMWARE_H__

#include <stdint.h>

#include "uart_boot.h"

extern int firmware_get_info(firmware_header_t* fh, const char* bin_file_path);
extern int firmware_read(uint8_t* data, uint32_t cursor, uint32_t size, const char* bin_file_path);

#endif //__FIRMWARE_H__
