#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <libusb.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>
#include <termios.h>
#include <sys/stat.h>
#include <pthread.h>
#include <signal.h>

#include "jpeglib.h"
#include "Palettes.h"

// FLIR camera definitions
#define VENDOR_ID 0x09cb
#define PRODUCT_ID 0x1996
#define RAMDISKPATH "/mnt/RAMDisk/"

// Power supply control
typedef struct {
	int serial_fd;
	double frequency;
	double voltage;
	double current;
	int modulation_active;
	int cycle_count;
	struct timeval start_time;
	char current_state[16];  // "ON", "OFF", "INIT"
} power_supply_t;

// Global variables
static struct libusb_device_handle *devh = NULL;
static power_supply_t ps = {-1, 0.5, 5.0, 0.1, 0, 0, {0}, "INIT"};
static int file_index = 0;
static int filecount = 0;
static struct timeval t1, t2;
static long long fps_t;
static int buf85pointer = 0;
static unsigned char buf85[1048576];
static volatile int acquisition_running = 1;
static char output_dir[256] = "";  // For copying files out of RAMDisk
static volatile int cleanup_in_progress = 0;  // Prevent double cleanup
static volatile int force_exit_counter = 0;   // Count Ctrl+C presses

// Function prototypes
int init_power_supply(const char* device, double voltage, double current);
void start_modulation(double frequency);
void* modulation_thread(void* arg);
void get_current_power_state(char *state_str, int *cycle_num);
void cleanup_and_exit(int sig);

int init_power_supply(const char* device, double voltage, double current) {
	printf("Initializing power supply: %s\n", device);

	ps.serial_fd = open(device, O_RDWR | O_NOCTTY);
	if (ps.serial_fd < 0) {
		fprintf(stderr, "Failed to open power supply device: %s (%s)\n", device, strerror(errno));
		return -1;
	}

	// Configure serial port for Korad power supply
	struct termios tty;
	tcgetattr(ps.serial_fd, &tty);

	// Set baud rate to 9600
	cfsetospeed(&tty, B9600);
	cfsetispeed(&tty, B9600);

	// Configure 8N1
	tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, local mode
	tty.c_cflag &= ~PARENB;             // No parity
	tty.c_cflag &= ~CSTOPB;             // 1 stop bit
	tty.c_cflag &= ~CSIZE;              // Clear size bits
	tty.c_cflag |= CS8;                 // 8 data bits

	// Disable flow control
	tty.c_cflag &= ~CRTSCTS;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

	// Raw mode
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty.c_oflag &= ~OPOST;

	// Set timeouts
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;

	tcsetattr(ps.serial_fd, TCSANOW, &tty);

	// Wait for power supply to stabilize
	usleep(100000);

	// Set voltage and current with proper Korad commands
	char cmd[64];

	// Set voltage
	snprintf(cmd, sizeof(cmd), "VSET1:%.2f", voltage);
	write(ps.serial_fd, cmd, strlen(cmd));
	usleep(100000);

	// Set current
	snprintf(cmd, sizeof(cmd), "ISET1:%.3f", current);
	write(ps.serial_fd, cmd, strlen(cmd));
	usleep(100000);

	// Turn output off initially
	write(ps.serial_fd, "OUT0", 4);
	usleep(100000);

	ps.voltage = voltage;
	ps.current = current;
	strcpy(ps.current_state, "OFF");

	printf("Power supply initialized: %.1fV, %.3fA\n", voltage, current);
	return 0;
}

void start_modulation(double frequency) {
	ps.frequency = frequency;
	ps.modulation_active = 1;
	ps.cycle_count = 0;
	gettimeofday(&ps.start_time, NULL);

	printf("Starting modulation: %.2f Hz\n", frequency);

	// Start modulation thread
	pthread_t mod_thread;
	pthread_create(&mod_thread, NULL, modulation_thread, NULL);
	pthread_detach(mod_thread);
}

void* modulation_thread(void* arg) {
	double period = 1.0 / ps.frequency;
	double on_time = period * 0.5;  // 50% duty cycle
	double off_time = period * 0.5;

	int cycle = 0;
	while (ps.modulation_active && acquisition_running) {
		// Turn ON
		int bytes_written = write(ps.serial_fd, "OUT1", 4);
		if (bytes_written >= 0) {
			strcpy(ps.current_state, "ON");
			ps.cycle_count = cycle;
		}

		usleep((int)(on_time * 1000000));

		if (!ps.modulation_active || !acquisition_running) break;

		// Turn OFF
		bytes_written = write(ps.serial_fd, "OUT0", 4);
		if (bytes_written >= 0) {
			strcpy(ps.current_state, "OFF");
		}

		cycle++;
		usleep((int)(off_time * 1000000));
	}

	// Ensure output is off when stopping
	write(ps.serial_fd, "OUT0", 4);
	strcpy(ps.current_state, "OFF");

	return NULL;
}

void get_current_power_state(char *state_str, int *cycle_num) {
	if (!ps.modulation_active) {
		strcpy(state_str, "OFF");
		*cycle_num = 0;
		return;
	}

	strcpy(state_str, ps.current_state);
	*cycle_num = ps.cycle_count;
}

void cleanup_and_exit(int sig) {
	printf("\nShutting down...\n");
	acquisition_running = 0;
	ps.modulation_active = 0;

	// Power supply cleanup first
	if (ps.serial_fd >= 0) {
		write(ps.serial_fd, "OUT0", 4);
		close(ps.serial_fd);
	}

	// Copy files if output directory specified
	if (strlen(output_dir) > 0) {
		printf("Copying files from RAMDisk to %s...\n", output_dir);
		char cmd[512];

		snprintf(cmd, sizeof(cmd), "mkdir -p '%s'", output_dir);
		system(cmd);

		snprintf(cmd, sizeof(cmd), "cp %sthermal_gray_*.pgm '%s/' 2>/dev/null || true", 
				RAMDISKPATH, output_dir);
		system(cmd);

		snprintf(cmd, sizeof(cmd), "ls -1 '%s'/thermal_gray_*.pgm 2>/dev/null | wc -l", output_dir);
		FILE *fp = popen(cmd, "r");
		if (fp) {
			char count_str[32];
			if (fgets(count_str, sizeof(count_str), fp)) {
				int copied_count = atoi(count_str);
				printf("Copied %d files to %s\n", copied_count, output_dir);
			}
			pclose(fp);
		}
	}

	printf("Acquisition complete: %d frames captured\n", filecount);

	// USB cleanup
	if (devh) {
		for (int i = 0; i <= 2; i++) {
			libusb_release_interface(devh, i);
		}
		libusb_close(devh);
		devh = NULL;
	}

	exit(0);
}

// RAW 16-BIT THERMAL DATA VERSION - Preserves full bit depth for scientific analysis
void vframe(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[]) {
	time_t now1;
	now1 = time(NULL);

	if (r < 0) {
		if (strcmp(EP_error, libusb_error_name(r)) != 0) {
			strcpy(EP_error, libusb_error_name(r));
			fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s\n", 
					ctime(&now1), ep, r, libusb_error_name(r));
			sleep(1);
		}
		return;
	}

	// Reset buffer if new chunk begins with magic bytes or buffer overflow
	unsigned char magicbyte[4] = {0xEF, 0xBE, 0x00, 0x00};

	if ((strncmp((char*)buf, (char*)magicbyte, 4) == 0) || ((buf85pointer + actual_length) >= 1048576)) {
		buf85pointer = 0;
	}

	memmove(buf85 + buf85pointer, buf, actual_length);
	buf85pointer = buf85pointer + actual_length;

	if (strncmp((char*)buf85, (char*)magicbyte, 4) != 0) {
		buf85pointer = 0;
		return;
	}

	// Parse frame header
	uint32_t FrameSize = buf85[8] + (buf85[9] << 8) + (buf85[10] << 16) + (buf85[11] << 24);
	uint32_t ThermalSize = buf85[12] + (buf85[13] << 8) + (buf85[14] << 16) + (buf85[15] << 24);
	uint32_t JpgSize = buf85[16] + (buf85[17] << 8) + (buf85[18] << 16) + (buf85[19] << 24);
	uint32_t StatusSize = buf85[20] + (buf85[21] << 8) + (buf85[22] << 16) + (buf85[23] << 24);

	if ((FrameSize + 28) > buf85pointer) {
		return; // Wait for more data
	}

	// Frame timing
	t1 = t2;
	gettimeofday(&t2, NULL);
	fps_t = (19 * fps_t + 10000000 / (((t2.tv_sec * 1000000) + t2.tv_usec) - 
				((t1.tv_sec * 1000000) + t1.tv_usec))) / 20;

	filecount++;

	// Get power state
	char power_state[16];
	int cycle_number;
	get_current_power_state(power_state, &cycle_number);

	// Calculate relative time since modulation start
	double relative_time = 0.0;
	if (ps.modulation_active) {
		relative_time = (t2.tv_sec - ps.start_time.tv_sec) + 
			(t2.tv_usec - ps.start_time.tv_usec) / 1000000.0;
	}

	// Print progress
	printf("#%06i %lld/10 fps, Power: %s, Cycle: %03d, T+%.3fs:", 
			filecount, fps_t, power_state, cycle_number, relative_time);
	for (int i = 0; i < StatusSize; i++) {
		int v = 28 + ThermalSize + JpgSize + i;
		if (buf85[v] > 31) {
			printf("%c", buf85[v]);
		}
	}
	printf("\n");

	buf85pointer = 0;

	// Allocate memory for the 16-bit raw pixel data
	unsigned short pix[160 * 120];

	// Extract raw 16-bit thermal data
	for (int y = 0; y < 120; ++y) {
		for (int x = 0; x < 160; ++x) {
			int offset;
			if (x < 80) {
				offset = 2 * (y * 164 + x) + 32;
			} else {
				offset = 2 * (y * 164 + x) + 32 + 4;
			}
			pix[y * 160 + x] = buf85[offset] + (buf85[offset + 1] << 8);
		}
	}

	// Find min/max values from the raw data
	unsigned short min = 0xFFFF, max = 0;
	for (int i = 0; i < 160 * 120; ++i) {
		if (pix[i] < min) min = pix[i];
		if (pix[i] > max) max = pix[i];
	}

	// Save the raw 16-bit PGM file
	char filename[200];
	sprintf(filename, RAMDISKPATH "thermal_gray_%06d.pgm", filecount);
	FILE *outfile = fopen(filename, "wb");
	if (outfile) {
		// PGM Header
		fprintf(outfile, "P5\n");
		fprintf(outfile, "# frame_number: %06i\n", filecount);
		fprintf(outfile, "# timestamp: %ld\n", t2.tv_sec);
		fprintf(outfile, "# timestamp_usec: %ld\n", t2.tv_usec);
		fprintf(outfile, "# power_state: %s\n", power_state);
		fprintf(outfile, "# cycle_number: %d\n", cycle_number);
		fprintf(outfile, "# relative_time: %.6f\n", relative_time);
		fprintf(outfile, "# modulation_freq: %.3f\n", ps.frequency);
		fprintf(outfile, "# voltage: %.3f\n", ps.voltage);
		fprintf(outfile, "# current: %.3f\n", ps.current);
		fprintf(outfile, "# min_temp_raw: %u\n", min);
		fprintf(outfile, "# max_temp_raw: %u\n", max);
		fprintf(outfile, "# status: ");
		for (int i = 0; i < StatusSize; i++) {
			int v = 28 + ThermalSize + JpgSize + i;
			if (buf85[v] > 31) {
				fprintf(outfile, "%c", buf85[v]);
			}
		}
		fprintf(outfile, "\n");

		fprintf(outfile, "160 120\n65535\n");
		fwrite(pix, sizeof(unsigned short), 160 * 120, outfile);
		fclose(outfile);
	}
}

static int find_lvr_flirusb(void) {
	devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
	return devh ? 0 : -EIO;
}

void print_bulk_result(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[]) {
	if (r < 0) {
		if (strcmp(EP_error, libusb_error_name(r)) != 0) {
			strcpy(EP_error, libusb_error_name(r));
			time_t now1 = time(NULL);
			fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s\n", 
					ctime(&now1), ep, r, libusb_error_name(r));
			sleep(1);
		}
	}
}

int EPloop(void) {
	int r = libusb_init(NULL);
	if (r < 0) {
		fprintf(stderr, "Failed to initialise libusb\n");
		exit(1);
	}

	r = find_lvr_flirusb();
	if (r < 0) {
		fprintf(stderr, "Could not find/open FLIR device\n");
		goto out;
	}
	printf("Found FLIR One G2 device\n");

	r = libusb_set_configuration(devh, 3);
	if (r < 0) {
		fprintf(stderr, "libusb_set_configuration error %d\n", r);
		goto out;
	}

	// Claim interfaces
	for (int i = 0; i <= 2; i++) {
		r = libusb_claim_interface(devh, i);
		if (r < 0) {
			fprintf(stderr, "libusb_claim_interface %d error %d\n", i, r);
			goto out;
		}
	}

	unsigned char buf[1048576];
	int actual_length;
	char EP81_error[50] = "", EP83_error[50] = "", EP85_error[50] = "";
	unsigned char data[2] = {0, 0};
	int state = 1;

	while (acquisition_running) {
		switch (state) {
			case 1:
				libusb_control_transfer(devh, 1, 0x0b, 0, 2, data, 0, 100);
				libusb_control_transfer(devh, 1, 0x0b, 0, 1, data, 0, 100);
				libusb_control_transfer(devh, 1, 0x0b, 1, 1, data, 0, 100);
				state = 2;
				break;

			case 2:
				// Send configuration commands
				unsigned char cmd1[16] = {0xcc,0x01,0x00,0x00,0x01,0x00,0x00,0x00,0x41,0x00,0x00,0x00,0xF8,0xB3,0xF7,0x00};
				libusb_bulk_transfer(devh, 2, cmd1, 16, &actual_length, 0);

				char json_cmd[] = "{\"type\":\"openFile\",\"data\":{\"mode\":\"r\",\"path\":\"CameraFiles.zip\"}}";
				libusb_bulk_transfer(devh, 2, (unsigned char*)json_cmd, strlen(json_cmd)+1, &actual_length, 0);

				unsigned char cmd2[16] = {0xcc,0x01,0x00,0x00,0x01,0x00,0x00,0x00,0x33,0x00,0x00,0x00,0xef,0xdb,0xc1,0xc1};
				libusb_bulk_transfer(devh, 2, cmd2, 16, &actual_length, 0);

				char json_cmd2[] = "{\"type\":\"readFile\",\"data\":{\"streamIdentifier\":10}}";
				libusb_bulk_transfer(devh, 2, (unsigned char*)json_cmd2, strlen(json_cmd2)+1, &actual_length, 0);

				state = 3;
				break;

			case 3:
				libusb_control_transfer(devh, 1, 0x0b, 1, 2, data, 2, 200);
				state = 4;
				break;

			case 4:
				// Main acquisition loop
				r = libusb_bulk_transfer(devh, 0x85, buf, sizeof(buf), &actual_length, 100);
				if (actual_length > 0) {
					vframe("0x85", EP85_error, r, actual_length, buf);
				}
				break;
		}

		// Poll other endpoints
		libusb_bulk_transfer(devh, 0x81, buf, sizeof(buf), &actual_length, 10);
		print_bulk_result("0x81", EP81_error, r, actual_length, buf);

		r = libusb_bulk_transfer(devh, 0x83, buf, sizeof(buf), &actual_length, 10);
		if (strcmp(libusb_error_name(r), "LIBUSB_ERROR_NO_DEVICE") == 0) {
			fprintf(stderr, "EP 0x83 LIBUSB_ERROR_NO_DEVICE -> reset USB\n");
			goto out;
		}
		print_bulk_result("0x83", EP83_error, r, actual_length, buf);
	}

out:
	if (devh) {
		for (int i = 0; i <= 2; i++) {
			libusb_release_interface(devh, i);
		}
		libusb_close(devh);
		devh = NULL;
	}
	return 0;
}

int main(int argc, char *argv[]) {
	// Parse command line arguments
	double frequency = 0.5;
	double voltage = 5.0;
	double current = 0.1;
	int duration = 30;
	char power_device[256] = "/dev/ttyACM0";

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--freq") == 0 && i + 1 < argc) {
			frequency = atof(argv[++i]);
		} else if (strcmp(argv[i], "--voltage") == 0 && i + 1 < argc) {
			voltage = atof(argv[++i]);
		} else if (strcmp(argv[i], "--current") == 0 && i + 1 < argc) {
			current = atof(argv[++i]);
		} else if (strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
			duration = atoi(argv[++i]);
		} else if (strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
			strcpy(power_device, argv[++i]);
		} else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
			strcpy(output_dir, argv[++i]);
		} else if (strcmp(argv[i], "--help") == 0) {
			printf("RAW 16-BIT FLIR8K with Integrated Power Supply Control\n");
			printf("======================================================\n");
			printf("Usage: %s [options]\n", argv[0]);
			printf("Options:\n");
			printf("  --freq <Hz>       Modulation frequency (default: 0.5)\n");
			printf("  --voltage <V>     Power supply voltage (default: 5.0)\n");
			printf("  --current <A>     Current limit (default: 0.1)\n");
			printf("  --duration <s>    Acquisition duration (default: 30)\n");
			printf("  --device <path>   Power supply device (default: /dev/ttyACM0)\n");
			printf("  --output <dir>    Copy files to directory when done\n");
			printf("  --help           Show this help\n");
			printf("\nSaves RAW 16-bit thermal data with synchronized power control\n");
			return 0;
		}
	}

	// Setup signal handlers
	signal(SIGINT, cleanup_and_exit);
	signal(SIGTERM, cleanup_and_exit);

	printf("RAW 16-BIT FLIR8K with Integrated Power Supply Control\n");
	printf("======================================================\n");
	printf("Configuration:\n");
	printf("  Modulation: %.2f Hz\n", frequency);
	printf("  Voltage: %.1f V\n", voltage);
	printf("  Current: %.3f A\n", current);
	printf("  Duration: %d seconds\n", duration);
	printf("  Power device: %s\n", power_device);
	if (strlen(output_dir) > 0) {
		printf("  Output directory: %s\n", output_dir);
	}

	// Check RAMDisk
	if (access(RAMDISKPATH, F_OK) != 0) {
		printf("RAMDisk not found at %s\n", RAMDISKPATH);
		printf("Please create and mount RAMDisk:\n");
		printf("  sudo mkdir -p %s\n", RAMDISKPATH);
		printf("  sudo mount -t tmpfs -o size=50m none %s\n", RAMDISKPATH);
		printf("  sudo chmod 777 %s\n", RAMDISKPATH);
		return 1;
	}

	// Clear previous files
	char cleanup_cmd[256];
	snprintf(cleanup_cmd, sizeof(cleanup_cmd), "rm -f %s*.pgm %s*.jpg 2>/dev/null || true", RAMDISKPATH, RAMDISKPATH);
	system(cleanup_cmd);

	// Initialize power supply
	if (init_power_supply(power_device, voltage, current) < 0) {
		fprintf(stderr, "Failed to initialize power supply\n");
		return 1;
	}

	// Reset frame counters
	filecount = 0;
	file_index = 0;

	// Start modulation
	start_modulation(frequency);

	// Start acquisition
	printf("Starting thermal acquisition...\n");

	// Set acquisition duration alarm
	if (duration > 0) {
		alarm(duration);
		signal(SIGALRM, cleanup_and_exit);
	}

	// Main acquisition loop
	while (acquisition_running) {
		EPloop();
	}

	cleanup_and_exit(0);
	return 0;
}
