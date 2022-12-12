#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "firmware.h"

int firmware_get_info(firmware_header_t* fh, const char* bin_file_path) {
	int index;
	uint32_t temp_data;
	uint32_t check_sum = 0;
	struct stat file_info;
	int binary_file = -1;

	binary_file = open(bin_file_path, O_RDONLY);
	if (binary_file < 0) {
		return -1;
	}

	fstat(binary_file, &file_info);

	for (index = 0; index < file_info.st_size; index += sizeof(uint32_t)) {
		temp_data = 0;
		pread(binary_file, &temp_data, sizeof(uint32_t), index);
		check_sum += temp_data;
	}

	close(binary_file);

	fh->bin_len = file_info.st_size;
	fh->checksum = (check_sum & 0xFFFF);

	return 0;
}

int firmware_read(uint8_t* data, uint32_t cursor, uint32_t size, const char* bin_file_path) {
	struct stat file_info;
	int binary_file = -1;

	binary_file = open(bin_file_path, O_RDONLY);
	if (binary_file < 0) {
		return -1;
	}

	fstat(binary_file, &file_info);

	if ((cursor + size) > (uint32_t)file_info.st_size) {
		return -1;
	}

	pread(binary_file, data, size, cursor);

	close(binary_file);

	return 0;
}
