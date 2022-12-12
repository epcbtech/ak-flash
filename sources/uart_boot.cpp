/**
 ******************************************************************************
 * @author: GaoKong
 * @date:   15/09/2017
 * @brief:  Host UART for AK kernel bootloader
 ******************************************************************************
**/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>

#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <sys/time.h>

#include "uart_boot.h"
#include "firmware.h"

using namespace std;

//#define FRAME_DEBUG_EN

#define CLOCKID					CLOCK_REALTIME
#define TIMER_1MS_SIG			SIGUSR1

#define TIMER_ADD_SIG			0x01
static timer_t timer_id_1s;

#define PBSTR		"############################################################"
#define PBWIDTH		60

typedef struct {
	firmware_header_t fw_header;
	int transfer;
} transfer_fw_status_t;

uint8_t frame_SYSTEM_AK_FLASH_UPDATE_REQ[] = { \
	/* header */
	0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x68, 0x32, 0xC2, \
	/* data */
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x69, 0x01, 0x00, 0x12, \
	0x00, 0x7B, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x06, 0x00, \
	0x80, 0xFF, 0x01, 0x0A, 0x80, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

/* help string */
static string help_string("example: ak_flash /dev/ttyUSB0 build/file_name.bin");

/* firmware file */
static string firmware_path;
static firmware_header_t file_firmware_header;

/* serial device */
static int uart_boot_dev_fd;	/* uart serial dev path */
static string uart_boot_dev_path;
int uart_boot_dev_opentty(const char* devpath);
uint32_t target_des_addr;

static pthread_t uart_boot_rx_thread;
static void* uart_boot_rx_thread_handler(void*);

static uart_boot_frame_t uart_boot_frame;

static uint8_t uart_boot_calcfcs(uint8_t len, uint8_t *data_ptr);
static void rx_frame_parser(uint8_t* data, uint8_t len);
static void tx_frame_post(uart_boot_data_cmd_t* cmd);

typedef struct {
	pthread_mutex_t mt;
	int32_t counter;
	uint32_t cmd;
	uint8_t enable;
} uart_boot_to_t;

static uart_boot_to_t uart_boot_to;

static void uart_boot_set_to(uint32_t ms, uint32_t cmd);
static void uart_boot_clear_to();


#define UART_BOOT_CMD_HANDSHAKE_REQ_SIG_TO			0x02
#define UART_BOOT_CMD_UPDATE_REQ_SIG_TO				0x03
#define UART_BOOT_CMD_TRANSFER_FW_REQ_SIG_TO		0x04
#define UART_BOOT_CMD_CHECKSUM_FW_REQ_SIG_TO		0x05

#define UART_BOOT_CMD_HANDSHAKE_REQ_SIG_INTERVAL	2
#define UART_BOOT_CMD_UPDATE_REQ_SIG_INTERVAL		2
#define UART_BOOT_CMD_TRANSFER_FW_REQ_SIG_INTERVAL	2
#define UART_BOOT_CMD_CHECKSUM_FW_REQ_SIG_INTERVAL	5

#define HANDSHAKE_REQ_RETRY_COUNTER_MAX				5 /* ~ 10s */

void uart_boot_cmd_handshake_req(void*);
void uart_boot_cmd_handshake_res(void*);
void uart_boot_cmd_update_req(void*);
void uart_boot_cmd_update_res(void*);
void uart_boot_cmd_transfer_fw_req(void*);
void uart_boot_cmd_transfer_fw_res(void*);
void uart_boot_cmd_checksum_fw_req(void*);
void uart_boot_cmd_checksum_fw_res(void*);

void timer_handler(int sig, siginfo_t *si, void *uc);

void print_progress (double percentage, transfer_fw_status_t* fw_stt) {
	int val = (int) (percentage * 100);
	int lpad = (int) (percentage * PBWIDTH);
	int rpad = PBWIDTH - lpad;
	printf("\rTotal: %6d bs\tTransfer: %6d bs\t%3d%% [%.*s%*s]",  fw_stt->fw_header.bin_len, fw_stt->transfer, val, lpad, PBSTR, rpad, "");
	fflush(stdout);
}

static transfer_fw_status_t transfer_fw_status;
static uint32_t handshake_req_retry_counter;

int main(int argc, char *argv[]) {
#if 0
	for (int i = 0; i < argc; i++) {
		cout << "argv[" << i << "]" << argv[i] << endl;
	}
#endif
	if (argc < 4) {
		cout << "[ERR] " << "please check parameter" << endl;
		cout << help_string << endl;
		return -1;
	}

	/***
	 * command to update firmware:
	 * host_uart_boot device_path file_path start_flash_address
	 * example: host_uart_boot /dev/ttyUSB0 build/file_name.bin 0x80002000
	 */

	/***
	 * check firmware file
	 */
	firmware_path.assign(argv[2]);

	if (firmware_get_info(&file_firmware_header, firmware_path.c_str()) == 0) {
		cout << "[OK] " << "file_firmware_header.psk		:" << file_firmware_header.psk		<< endl;
		cout << "[OK] " << "file_firmware_header.bin_len	:" << file_firmware_header.bin_len	<< endl;
		cout << "[OK] " << "file_firmware_header.checksum	:" << std::hex << file_firmware_header.checksum	<< endl;

		memcpy(&transfer_fw_status.fw_header, &file_firmware_header, sizeof(firmware_header_t));
	}
	else {
		cout << "[ERR] " << "file: " << firmware_path << " is not found." << endl;
		return -1;
	}

	target_des_addr = ((uint32_t)strtol(argv[3], NULL, 0));
	printf("[OK] target_des_addr: 0x%08X\n", target_des_addr);

	/* configure timer */
	struct sigevent sev;
	struct itimerspec its;
	struct sigaction sa;

	sa.sa_flags		= SA_SIGINFO;
	sa.sa_sigaction	= timer_handler;
	sigemptyset(&sa.sa_mask);
	sigaction(TIMER_1MS_SIG, &sa, NULL);

	sev.sigev_notify		= SIGEV_SIGNAL;
	sev.sigev_signo			= TIMER_1MS_SIG;
	sev.sigev_value.sival_ptr	= &timer_id_1s;
	timer_create(CLOCKID, &sev, &timer_id_1s);

	its.it_value.tv_sec		= 1; /* timer 1s */
	its.it_value.tv_nsec	= 0;
	its.it_interval.tv_sec	= its.it_value.tv_sec;
	its.it_interval.tv_nsec	= its.it_value.tv_nsec;
	timer_settime(timer_id_1s, 0, &its, NULL);

	/***
	 * check device path
	 */
	uart_boot_dev_path.assign(argv[1]);

	if (uart_boot_dev_opentty(uart_boot_dev_path.c_str()) < 0) {
		cout << "[ERR] " << "Can't open dev path:" << uart_boot_dev_path.c_str() << endl;
		exit(-1);
	}
	else {
		pthread_create(&uart_boot_rx_thread, NULL, uart_boot_rx_thread_handler, NULL);
	}

	while (1) {}

	return 0;
}

void timer_handler(int sig, siginfo_t *si, void *uc) {
	(void)sig;
	(void)si;
	(void)uc;

	uint8_t clear_to_flag = 1;

	pthread_mutex_lock(&uart_boot_to.mt);

	if (uart_boot_to.enable) {
		if (uart_boot_to.counter-- <= 0) {
			switch (uart_boot_to.cmd) {

			case UART_BOOT_CMD_HANDSHAKE_REQ_SIG_TO: {
				if (handshake_req_retry_counter++ < HANDSHAKE_REQ_RETRY_COUNTER_MAX) {
					clear_to_flag = 0;

					pthread_mutex_unlock(&uart_boot_to.mt);
					uart_boot_cmd_handshake_req(NULL);
					pthread_mutex_lock(&uart_boot_to.mt);

					cout << "\r[WRN] " << "handshake retry times " << std::dec << handshake_req_retry_counter << "!" << endl;
				}
				else {
					cout << "[ERR] " << "handshake faulted !" << endl;
					cout << "[ERR] " << "Please check boot condition !" << endl;
					exit(-1);
				}
			}
				break;

			case UART_BOOT_CMD_UPDATE_REQ_SIG_TO: {
				cout << "[ERR] " << "UART_BOOT_CMD_UPDATE_REQ_SIG_TO" << endl;
				exit(-1);
			}
				break;

			case UART_BOOT_CMD_TRANSFER_FW_REQ_SIG_TO: {
				cout << "[ERR] " << "UART_BOOT_CMD_TRANSFER_FW_REQ_SIG_TO" << endl;
				exit(-1);
			}
				break;

			case UART_BOOT_CMD_CHECKSUM_FW_REQ_SIG_TO: {
				cout << "[ERR] " << "UART_BOOT_CMD_CHECKSUM_FW_REQ_SIG_TO" << endl;
				exit(-1);
			}
				break;

			default:
				break;
			}

			/* clear timeout */
			if (clear_to_flag) {
				uart_boot_to.enable = 0;
				uart_boot_to.cmd = 0;
				uart_boot_to.counter = 0;
			}
		}
	}

	pthread_mutex_unlock(&uart_boot_to.mt);
}

void uart_boot_set_to(uint32_t ms, uint32_t cmd) {
	pthread_mutex_lock(&uart_boot_to.mt);
	uart_boot_to.enable = 1;
	uart_boot_to.counter = ms;
	uart_boot_to.cmd = cmd;
	pthread_mutex_unlock(&uart_boot_to.mt);
}

void uart_boot_clear_to() {
	pthread_mutex_lock(&uart_boot_to.mt);
	uart_boot_to.enable = 0;
	uart_boot_to.cmd = 0;
	uart_boot_to.counter = 0;
	pthread_mutex_unlock(&uart_boot_to.mt);
}

void uart_boot_init(pf_uart_boot_cmd_handler boot_entry_handler) {
	/* sortware init */
	uart_boot_frame.index	= 0;
	uart_boot_frame.len		= 0;
	uart_boot_frame.state	= SOP_STATE;
	uart_boot_frame.uart_boot_cmd_handler = boot_entry_handler;
}

void extract_complete_lines(std::string &buf, std::vector<std::string> &lines) {
	std::string::size_type pos;
	while ((pos = buf.find ('\n')) != std::string::npos) {
		lines.push_back (buf.substr (0, pos));
		buf.erase (0, pos + 1);
	}
}

int uart_boot_dev_opentty(const char* devpath) {
	char readbuf[256];
	FILE *st_sys_ret_fp;
	struct termios options;
	string st_realdevpath;
	string st_devconflict;
	string command;

	/* Check conflick device */
	command.assign("readlink -f ");
	command.append(devpath);
	st_sys_ret_fp = popen(command.c_str(), "r");

	st_realdevpath.assign("");

	if (st_sys_ret_fp != NULL) {
		do {
			memset(readbuf, 255, 0);
			fgets(readbuf, 254, st_sys_ret_fp);
			if(feof(st_sys_ret_fp)) break;

			st_realdevpath.append(string((const char*)readbuf));
			st_realdevpath.erase(std::remove(st_realdevpath.begin(), st_realdevpath.end(), '\n'), st_realdevpath.end());
		} while(!feof(st_sys_ret_fp));

		pclose(st_sys_ret_fp);
	}
	else {
		cout << "[ERR] " << "popen( " << "readlink -f " << devpath << " )" << endl;
		exit(1);
	}

	st_devconflict.assign("");

	if (st_realdevpath.compare(devpath) == 0) { /* using real device name */
		command.assign("ps -ef | grep  ");
		command.append(st_realdevpath);
		st_sys_ret_fp = popen(command.c_str(), "r");

		if (st_sys_ret_fp != NULL) {
			do {
				memset(readbuf, 255, 0);
				fgets(readbuf, 254, st_sys_ret_fp);
				if(feof(st_sys_ret_fp)) break;

				st_devconflict.append(string((const char*)readbuf));
			} while(!feof(st_sys_ret_fp));

			pclose(st_sys_ret_fp);
		}
		else {
			cout << "[ERR] " << "popen( " << "readlink -f " << st_realdevpath << " )" << endl;
			exit(1);
		}

		std::vector<std::string> vector_ret_lines;
		extract_complete_lines(st_devconflict, vector_ret_lines);

		if (vector_ret_lines.size() > 3) {
			cout << "\n[ERR] " << "Device is busy now, Please check !\n" << endl;
			for (auto line : vector_ret_lines) {
				std::cout << line << '\n';
			}
			cout << "\n[HELP] " << "Using command fuser -k " << st_realdevpath << endl << endl;
			exit(1);
		}
	}
	else { /* using alias device name */
		command.assign("ps -ef | grep  ");
		command.append(devpath);
		st_sys_ret_fp = popen(command.c_str(), "r");

		if (st_sys_ret_fp != NULL) {
			do {
				memset(readbuf, 255, 0);
				fgets(readbuf, 254, st_sys_ret_fp);
				if(feof(st_sys_ret_fp)) break;

				st_devconflict.append(string((const char*)readbuf));
			} while(!feof(st_sys_ret_fp));

			pclose(st_sys_ret_fp);
		}
		else {
			cout << "[ERR] " << "popen( " << "readlink -f " << devpath << " )" << endl;
			exit(1);
		}

		std::vector<std::string> vector_ret_lines;
		extract_complete_lines(st_devconflict, vector_ret_lines);

		if (vector_ret_lines.size() > 3) {
			cout << "\n[ERR] " << "Device is busy now, Please check !\n" << endl;
			for (auto line : vector_ret_lines) {
				std::cout << line << '\n';
			}
			cout << "\n[HELP] " << "Using command fuser -k " << devpath << endl << endl;
			exit(1);
		}

		command.assign("ps -ef | grep  ");
		command.append(st_realdevpath);
		st_sys_ret_fp = popen(command.c_str(), "r");
		st_devconflict.assign("");

		if (st_sys_ret_fp != NULL) {
			do {
				memset(readbuf, 255, 0);
				fgets(readbuf, 254, st_sys_ret_fp);
				if(feof(st_sys_ret_fp)) break;

				st_devconflict.append(string((const char*)readbuf));
			} while(!feof(st_sys_ret_fp));

			pclose(st_sys_ret_fp);
		}
		else {
			cout << "[ERR] " << "popen( " << "readlink -f " << st_realdevpath << " )" << endl;
			exit(1);
		}

		vector_ret_lines.clear();
		extract_complete_lines(st_devconflict, vector_ret_lines);

		if (vector_ret_lines.size() > 2) {
			cout << "\n[ERR] " << "Device is busy now, Please check !\n" << endl;
			for (auto line : vector_ret_lines) {
				std::cout << line << '\n';
			}
			cout << "\n[HELP] " << "Using command fuser -k " << st_realdevpath << endl << endl;
			exit(1);
		}
	}

	uart_boot_dev_fd = open(st_realdevpath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart_boot_dev_fd < 0) {
		return uart_boot_dev_fd;
	}
	else {
		fcntl(uart_boot_dev_fd, F_SETFL, 0);

		/* get current status */
		tcgetattr(uart_boot_dev_fd, &options);

		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);

		/* No parity (8N1) */
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;

		options.c_cflag |= (CLOCAL | CREAD);
		options.c_cflag     &=  ~CRTSCTS;

		cfmakeraw(&options);

		tcflush(uart_boot_dev_fd, TCIFLUSH);
		if (tcsetattr (uart_boot_dev_fd, TCSANOW, &options) != 0) {
			cout << "[ERR] " << "error in tcsetattr()" << endl;
			exit(-1);
		}
	}

	return uart_boot_dev_fd;
}

void rx_frame_parser(uint8_t* data, uint8_t len) {
	uint8_t ch;
	int rx_remain;

	while(len) {

		ch = *data++;
		len--;

		switch (uart_boot_frame.state) {
		case SOP_STATE: {
			if (UART_BOOT_SOP_CHAR == ch) {
				uart_boot_frame.state = LEN_STATE;
			}
		}
			break;

		case LEN_STATE: {
			if (ch > UART_BOOT_FRAME_DATA_SIZE) {
				uart_boot_frame.state = SOP_STATE;
				return;
			}
			else {
				uart_boot_frame.len = ch;
				uart_boot_frame.index = 0;
				uart_boot_frame.state = DATA_STATE;
			}
		}
			break;

		case DATA_STATE: {
			uart_boot_frame.data[uart_boot_frame.index++] = ch;

			rx_remain = uart_boot_frame.len - uart_boot_frame.index;

			if (len >= rx_remain) {
				memcpy((uint8_t*)(uart_boot_frame.data + uart_boot_frame.index), data, rx_remain);
				uart_boot_frame.index += rx_remain;
				len -= rx_remain;
				data += rx_remain;
			}
			else {
				memcpy((uint8_t*)(uart_boot_frame.data + uart_boot_frame.index), data, len);
				uart_boot_frame.index += len;
				len = 0;
			}

			if (uart_boot_frame.index == uart_boot_frame.len) {
				uart_boot_frame.state = FCS_STATE;
			}
		}
			break;

		case FCS_STATE: {
			uart_boot_frame.state = SOP_STATE;

			uart_boot_frame.fcs = ch;

			if (uart_boot_frame.fcs \
					== uart_boot_calcfcs(uart_boot_frame.len, uart_boot_frame.data)) {

#if defined(FRAME_DEBUG_EN)
				cout << "[OK] " << "checksum correctly !" << endl;
				cout << "[RX_DATA] [";
				for (int i = 0; i < uart_boot_frame.len; i++) {
					cout << ' ' << std::hex << uart_boot_frame.data[i];
				}
				cout << " ]" << endl;
#endif

				uart_boot_frame.uart_boot_cmd_handler((uart_boot_frame_t*)&uart_boot_frame);
			}
			else {
				/* TODO: handle checksum incorrect */
				cout << "[ERR] " << "checksum incorrectly !" << endl;
				exit(-1);
			}
		}
			break;

		default:
			break;
		}
	}
}

uint8_t uart_boot_calcfcs(uint8_t len, uint8_t *data_ptr) {
	uint8_t xor_result;
	xor_result = len;
	for (int i = 0; i < len; i++, data_ptr++) {
		xor_result = xor_result ^ *data_ptr;
	}
	return xor_result;
}

void tx_frame_post(uart_boot_data_cmd_t* cmd) {
	uint8_t len, sop, fcs;

	/* SOP */
	sop = UART_BOOT_SOP_CHAR;
	write(uart_boot_dev_fd, &sop, 1);

	/* len */
	len = sizeof(uart_boot_cmd_t) + 1 + cmd->len;
	write(uart_boot_dev_fd, &len, 1);

	/* data */
	write(uart_boot_dev_fd, cmd, len);

	/* FCS */
	fcs = uart_boot_calcfcs(len, (uint8_t*)cmd);
	write(uart_boot_dev_fd, &fcs, 1);

#if defined(FRAME_DEBUG_EN)
	printf("[START-FRAME ------------------]\n");
	printf("SOP: %02X\n", sop);
	printf("LEN: %d\n", len);
	printf("DAT: ");
	for (int i = 0; i < len; i++) {
		printf("%02X ", ((uint8_t*)cmd)[i]);
	}
	printf("\nFCS: %02X\n", fcs);
	printf("[STOP-FRAME  ------------------]\n");
#endif
}

#define RX_BUFFER_SIZE		4096
void* uart_boot_rx_thread_handler(void*) {
	uint8_t rx_buffer[RX_BUFFER_SIZE];
	uint32_t rx_read_len;

	const char* fwu = "fwu\r\n";
	write(uart_boot_dev_fd, fwu, strlen(fwu));
	usleep(200000);

	//	write(uart_boot_dev_fd, frame_SYSTEM_AK_FLASH_UPDATE_REQ, \
	//		  sizeof(frame_SYSTEM_AK_FLASH_UPDATE_REQ));
	// usleep(200000);
	// tcflush(uart_boot_dev_fd, TCIFLUSH);

	uart_boot_cmd_handshake_req(NULL);

	while(1) {
		rx_read_len = read(uart_boot_dev_fd, rx_buffer, RX_BUFFER_SIZE);
		if (rx_read_len > 0) {
			rx_frame_parser(rx_buffer, rx_read_len);
		}
	}

	return (void*)0;
}

void uart_boot_cmd_handshake_req(void* boot_obj) {
	uart_boot_data_cmd_t uart_boot_data_cmd;
	uart_boot_data_cmd.boot_cmd.cmd = UART_BOOT_CMD_HANDSHAKE_REQ;
	uart_boot_data_cmd.boot_cmd.subcmd = 0;

	firmware_boot_cmd_t firmware_boot_cmd;
	firmware_boot_cmd.cmd		= SYS_BOOT_CMD_NONE;
	firmware_boot_cmd.container	= SYS_BOOT_CONTAINER_DIRECTLY;
	firmware_boot_cmd.io_driver	= SYS_BOOT_IO_DRIVER_UART;
	firmware_boot_cmd.des_addr	= target_des_addr;
	firmware_boot_cmd.src_addr	= 0;
	memset(&firmware_boot_cmd.ak_msg_res, 0, sizeof(ak_msg_host_res_t));

	uart_boot_data_cmd.len = sizeof(firmware_boot_cmd_t);
	memcpy(uart_boot_data_cmd.data, &firmware_boot_cmd, sizeof(firmware_boot_cmd_t));

	tx_frame_post(&uart_boot_data_cmd);

	uart_boot_frame.uart_boot_cmd_handler = uart_boot_cmd_handshake_res;
	uart_boot_set_to(UART_BOOT_CMD_HANDSHAKE_REQ_SIG_INTERVAL, UART_BOOT_CMD_HANDSHAKE_REQ_SIG_TO);
}

void uart_boot_cmd_handshake_res(void* boot_obj) {
	uart_boot_clear_to();

	uart_boot_frame_t* uart_boot_frame = (uart_boot_frame_t*)boot_obj;
	uart_boot_data_cmd_t* boot_cmd = (uart_boot_data_cmd_t*)(uart_boot_frame->data);

	if (boot_cmd->boot_cmd.cmd == UART_BOOT_CMD_HANDSHAKE_RES) {
		uart_boot_cmd_update_req(NULL);
	}
	else {
		cout << "[ERR] " << "unexpected command !" << endl;
		exit(-1);
	}
}

void uart_boot_cmd_update_req(void*) {
	uart_boot_data_cmd_t uart_boot_data_cmd;

	uart_boot_data_cmd.boot_cmd.cmd = UART_BOOT_CMD_UPDATE_REQ;
	uart_boot_data_cmd.boot_cmd.subcmd = 0;
	uart_boot_data_cmd.len = sizeof(firmware_header_t);
	memcpy(uart_boot_data_cmd.data, &file_firmware_header, sizeof(firmware_header_t));

	tx_frame_post(&uart_boot_data_cmd);

	uart_boot_frame.uart_boot_cmd_handler = uart_boot_cmd_update_res;

	uart_boot_set_to(UART_BOOT_CMD_UPDATE_REQ_SIG_INTERVAL, UART_BOOT_CMD_UPDATE_REQ_SIG_TO);
}

void uart_boot_cmd_update_res(void* boot_obj) {
	uart_boot_clear_to();

	uart_boot_frame_t* p_uart_boot_frame = (uart_boot_frame_t*)boot_obj;
	uart_boot_data_cmd_t* boot_cmd = (uart_boot_data_cmd_t*)(p_uart_boot_frame->data);

	if (boot_cmd->boot_cmd.cmd == UART_BOOT_CMD_UPDATE_RES) {
		if (boot_cmd->boot_cmd.subcmd == UART_BOOT_SUB_CMD_1) {
			uint32_t erase_addr;
			memcpy(&erase_addr, boot_cmd->data, sizeof(uint32_t));
			printf("\rFlash page at addr: 0x%08x erased", erase_addr);
		}
		else if (boot_cmd->boot_cmd.subcmd == UART_BOOT_SUB_CMD_2) {
			printf("\n");
			uart_boot_cmd_transfer_fw_req(NULL);
			uart_boot_frame.uart_boot_cmd_handler = uart_boot_cmd_transfer_fw_res;
		}
		else {
			cout << "[ERR] " << "unexpected sub command !" << endl;
			exit(-1);
		}
	}
	else {
		cout << "[ERR] " << "unexpected command !" << endl;
		exit(-1);
	}
}

static uint8_t fw_frame_buffer[UART_BOOT_CMD_DATA_SIZE];
static uint8_t fw_frame_len = 0;
static uint8_t checksum_flag = 0;
static uint32_t firmware_transfer_remain = 0;
static uint32_t firmware_transfer_index = 0;

void uart_boot_cmd_transfer_fw_req(void*) {
	firmware_transfer_remain = file_firmware_header.bin_len - firmware_transfer_index;

	if (firmware_transfer_remain <= UART_BOOT_CMD_DATA_SIZE) {
		fw_frame_len = (uint8_t)firmware_transfer_remain;
	}
	else {
		fw_frame_len = UART_BOOT_CMD_DATA_SIZE;
	}

	firmware_read(fw_frame_buffer, firmware_transfer_index, fw_frame_len, firmware_path.c_str());
	firmware_transfer_index += fw_frame_len;

	if (firmware_transfer_index <= file_firmware_header.bin_len && checksum_flag == 0) {
		uart_boot_data_cmd_t uart_boot_data_cmd;
		uart_boot_data_cmd.boot_cmd.cmd		= UART_BOOT_CMD_TRANSFER_FW_REQ;
		uart_boot_data_cmd.boot_cmd.subcmd	= 0;
		uart_boot_data_cmd.len				= UART_BOOT_CMD_DATA_SIZE;

		memset(uart_boot_data_cmd.data, 0, UART_BOOT_CMD_DATA_SIZE);
		memcpy(uart_boot_data_cmd.data, fw_frame_buffer, fw_frame_len);

		tx_frame_post(&uart_boot_data_cmd);

		uart_boot_set_to(UART_BOOT_CMD_TRANSFER_FW_REQ_SIG_INTERVAL, UART_BOOT_CMD_TRANSFER_FW_REQ_SIG_TO);

		transfer_fw_status.transfer = firmware_transfer_index;

		double percent = ((double)transfer_fw_status.transfer / (double)transfer_fw_status.fw_header.bin_len);

		print_progress(percent, &transfer_fw_status);

		if ((int)percent == 1) {
			printf("\n");
		}

		if (firmware_transfer_index >= file_firmware_header.bin_len) checksum_flag = 1;
	}
	else {
		uart_boot_cmd_checksum_fw_req(NULL);
		uart_boot_frame.uart_boot_cmd_handler = uart_boot_cmd_checksum_fw_res;
	}
}

void uart_boot_cmd_transfer_fw_res(void* boot_obj) {
	uart_boot_clear_to();

	uart_boot_frame_t* uart_boot_frame = (uart_boot_frame_t*)boot_obj;
	uart_boot_cmd_t* boot_cmd = (uart_boot_cmd_t*)(uart_boot_frame->data);

	if (boot_cmd->cmd == UART_BOOT_CMD_TRANSFER_FW_RES) {
		uart_boot_cmd_transfer_fw_req(NULL);
	}
	else {
		cout << "[ERR] " << "unexpected command !" << endl;
		exit(-1);
	}
}

void uart_boot_cmd_checksum_fw_req(void*) {
	uart_boot_data_cmd_t uart_boot_data_cmd;
	uart_boot_data_cmd.boot_cmd.cmd		= UART_BOOT_CMD_CHECKSUM_FW_REQ;
	uart_boot_data_cmd.boot_cmd.subcmd	= 0;
	uart_boot_data_cmd.len				= 0;
	tx_frame_post(&uart_boot_data_cmd);

	uart_boot_set_to(UART_BOOT_CMD_CHECKSUM_FW_REQ_SIG_INTERVAL, UART_BOOT_CMD_CHECKSUM_FW_REQ_SIG_TO);
}

void uart_boot_cmd_checksum_fw_res(void* boot_obj) {
	uart_boot_clear_to();

	uart_boot_frame_t* uart_boot_frame = (uart_boot_frame_t*)boot_obj;
	uart_boot_cmd_t* boot_cmd = (uart_boot_cmd_t*)(uart_boot_frame->data);

	if (boot_cmd->cmd == UART_BOOT_CMD_CHECKSUM_FW_RES) {
		if (boot_cmd->subcmd == UART_BOOT_SUB_CMD_1) {
			cout << "[OK] " << "Update firmware successfully !" << endl;
		}
		else if (boot_cmd->subcmd == UART_BOOT_SUB_CMD_2) {
			cout << "[ERR] " << "Target checksum incorrectly !" << endl;
			exit(-1);
		}
		else {
			cout << "[ERR] " << "unexpected sub command !" << endl;
			exit(-1);
		}
	}
	else {
		cout << "[ERR] " << "unexpected command !" << endl;
		exit(-1);
	}

	exit(0);
}
