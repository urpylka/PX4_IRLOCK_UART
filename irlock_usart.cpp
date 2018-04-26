/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <px4_defines.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <drivers/device/ringbuffer.h>
#include <stdio.h>
#include <uORB/uORB.h>
#include <uORB/topics/irlock_report.h>
#include <termios.h>


#include <drivers/drv_hrt.h>
#include <drivers/drv_irlock.h>
#include <drivers/device/device.h>


#define IRLOCK_DEFAULT_PORT		"/dev/ttyS6"	// telem2 on Pixhawk
#define FRAME_LEN       18
#define HEADER_LEN      4
#define BUF_LEN 		54
#define GETINDEX(x)     x % BUF_LEN

#define IRLOCK_CONVERSION_INTERVAL_US	20000U /** us = 20ms = 50Hz **/

#define IRLOCK_SYNC			0xAA55AA55

#define IRLOCK_RES_X 320
#define IRLOCK_RES_Y 240

#define IRLOCK_CENTER_X				(IRLOCK_RES_X/2)			// the x-axis center pixel position
#define IRLOCK_CENTER_Y				(IRLOCK_RES_Y/2)			// the y-axis center pixel position

#define IRLOCK_FOV_X (70.8f*M_PI_F/180.0f)
#define IRLOCK_FOV_Y (55.6f*M_PI_F/180.0f)

#define IRLOCK_TAN_HALF_FOV_X 0.71066300938f // tan(0.5 * 60 * pi/180)
#define IRLOCK_TAN_HALF_FOV_Y 0.52724018875f // tan(0.5 * 35 * pi/180)

#define IRLOCK_TAN_ANG_PER_PIXEL_X	(2*IRLOCK_TAN_HALF_FOV_X/IRLOCK_RES_X)
#define IRLOCK_TAN_ANG_PER_PIXEL_Y	(2*IRLOCK_TAN_HALF_FOV_Y/IRLOCK_RES_Y)

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


extern "C" __EXPORT int irlock_main(int argc, char *argv[]);

class IRLOCK : public device::CDev
{
public:
	IRLOCK(const char *port = IRLOCK_DEFAULT_PORT);
	virtual ~IRLOCK();

	virtual int 			init();
	virtual int 			info();
	virtual int 			test();

	int				start();
	void 				stop();

private:
	uint8_t _rotation;
	bool				_task_should_exit;
	int 				_task_handle;
	char 				_port[20];
	int				_class_instance;
	int				_orb_class_instance;

	ringbuffer::RingBuffer *_reports;
	bool _sensor_ok;
	work_s _work;
	uint32_t _read_failures;
	orb_advert_t _irlock_report_topic;

	uint8_t 	_head;
	uint8_t 	_tail;
	uint8_t 	_buf[BUF_LEN];

	void read_device();

	/** static function that is called by worker queue, arg will be pointer to instance of this class **/
	static void	cycle_trampoline(int argc, char *argv[]);

	/** read from device and schedule next read **/
	void		cycle();

	bool read_and_parse(uint8_t *buf, int len, irlock_target_s *block);

	bool is_header(uint8_t c);
};

namespace
{
IRLOCK	*g_irlock = nullptr;
}

void irlock_usage();

IRLOCK::IRLOCK(const char *port) :
	CDev("irlock", IRLOCK0_DEVICE_PATH),
	_task_should_exit(false),
	_task_handle(-1),
	_class_instance(-1),
	_orb_class_instance(-1),
	_reports(nullptr),
	_sensor_ok(false),
	_read_failures(0),
	_irlock_report_topic(nullptr),
	_head(0),
	_tail(0)

{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	// disable debug() calls
	_debug_enabled = true;

	memset(&_buf[0], 0, sizeof(_buf));
	memset(&_work, 0, sizeof(_work));
}

IRLOCK::~IRLOCK()
{

	if (_class_instance != -1) {
		unregister_class_devname(IRLOCK0_DEVICE_PATH, _class_instance);
	}

	orb_unadvertise(_irlock_report_topic);

	if (_task_handle != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_task_handle);
				break;
			}
		} while (_task_handle != -1);
	}
	
	//work_cancel(HPWORK, &_work);
	/** clear reports queue **/
	if (_reports != nullptr) {
		delete _reports;
	}
}

int
IRLOCK::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		ret = CDev::init();

		if (ret != OK) {
			PX4_WARN("vdev init failed");
			break;
		}

		/* open fd */
		int fd = ::open(_port, O_RDWR | O_NOCTTY);
		PX4_INFO("passed open port");

		if (fd < 0) {
			PX4_WARN("failed to open serial device");
			ret = 1;
			break;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(fd, &uart_config);

		/** Input flags - Turn off input processing
		 *
		 * convert break to null byte, no CR to NL translation,
		 * no NL to CR translation, don't mark parity errors or breaks
		 * no input parity check, don't strip high bit off,
		 * no XON/XOFF software flow control
		 *
		 */

		uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
					 INLCR | PARMRK | INPCK | ISTRIP | IXON);

		/** No line processing
		 *
		 * echo off, echo newline off, canonical mode off,
		 * extended input processing off, signal chars off
		 *
		 */

		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag &= ~(CSTOPB | PARENB);



		unsigned speed = B115200;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d ISPD", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d OSPD\n", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
			PX4_WARN("ERR baud %d ATTR", termios_state);
			ret = 1;
			break;
		}

		::close(fd);

		/** allocate buffer storing values read from sensor **/
		_reports = new ringbuffer::RingBuffer(2, sizeof(struct irlock_s));

		if (_reports == nullptr) {
			return ENOTTY;

		} else {
			_sensor_ok = true;
			/** start work queue **/
			start();
			return OK;
		}

	} while (0);

	return ret;
}

int
IRLOCK::start()
{
	/** flush ring and reset state machine **/
	_reports->flush();

	/** start work queue cycle **/
	// work_queue(HPWORK, &_work, (worker_t)&IRLOCK::cycle_trampoline, this, 20);

	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("irlock",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX - 30,
					  800,
					  (px4_main_t)&IRLOCK::cycle_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

/** stop periodic reads from sensor **/
void IRLOCK::stop()
{
	//work_cancel(HPWORK, &_work);

	if (_task_handle != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_task_handle);
				break;
			}
		} while (_task_handle != -1);
	}
}

/** display driver info **/
int IRLOCK::info()
{
	if (g_irlock == nullptr) {
		errx(1, "irlock device driver is not running");
	}

	/** display reports in queue **/
	if (_sensor_ok) {
		_reports->print_info("report queue: ");
		warnx("read errors:%lu", (unsigned long)_read_failures);

	} else {
		warnx("sensor is not healthy");
	}

	return OK;
}

/** test driver **/
int IRLOCK::test()
{
	/** exit immediately if driver not running **/
	if (g_irlock == nullptr) {
		errx(1, "irlock device driver is not running");
	}

	/** exit immediately if sensor is not healty **/
	if (!_sensor_ok) {
		errx(1, "sensor is not healthy");
	}

	/** instructions to user **/
	warnx("searching for object for 10 seconds");

	/** read from sensor for 10 seconds **/
	struct irlock_s report;
	uint64_t start_time = hrt_absolute_time();

	while ((hrt_absolute_time() - start_time) < 10000000) {
		if (_reports->get(&report)) {
			/** output all objects found **/
			for (uint8_t i = 0; i < report.num_targets; i++) {
				warnx("sig:%d x:%4.3f y:%4.3f width:%4.3f height:%4.3f",
				      (int)report.targets[i].signature,
				      (double)report.targets[i].pos_x,
				      (double)report.targets[i].pos_y,
				      (double)report.targets[i].size_x,
				      (double)report.targets[i].size_y);
			}
		}

		/** sleep for 0.05 seconds **/
		usleep(50000);
	}

	return OK;
}

void IRLOCK::cycle_trampoline(int argc, char *argv[])
{
	// IRLOCK *device = (IRLOCK *)arg;

	/** check global irlock reference and cycle **/
	if (g_irlock != nullptr) {
		g_irlock->cycle();
	}
}

void IRLOCK::cycle()
{
	/** ignoring failure, if we do, we will be back again right away... **/
	read_device();

	/** schedule the next cycle **/
	// work_queue(HPWORK, &_work, (worker_t)&IRLOCK::cycle_trampoline, this, USEC2TICK(IRLOCK_CONVERSION_INTERVAL_US));
}

bool IRLOCK::is_header(uint8_t c) {
	static uint32_t header;
	header = header << 8 | ((uint32_t) c);
	return header == IRLOCK_SYNC;
}

bool IRLOCK::read_and_parse(uint8_t *buf, int len, irlock_target_s *block)
{
	bool ret = false;

	// write new data into a ring buffer
	for (uint8_t i = 0; i < len; i++) {

		_head++;

		if (_head >= BUF_LEN) {
			_head = 0;
		}

		if (_tail == _head) {
			_tail = (_tail == BUF_LEN - 1) ? 0 : _head + 1;
		}

		_buf[_head] = buf[i];
	}

	// check how many bytes are in the buffer, return if it's lower than the size of one package
	uint8_t num_bytes = _head >= _tail ? (_head - _tail + 1) : (_head + 1 + BUF_LEN - _tail);

	if (num_bytes < FRAME_LEN) {
		PX4_DEBUG("not enough bytes");
		return false;
	}

	int8_t index = _head;
	uint8_t no_header_counter = 0;	// counter for bytes which are non header bytes

	// go through the buffer backwards starting from the newest byte
	// if we find a header byte and the previous two bytes weren't header bytes
	// then we found the newest package.
	for (uint8_t i = 0; i < num_bytes; i++) {
		if (is_header(_buf[index])) {
			if (no_header_counter >= FRAME_LEN - HEADER_LEN) {
				uint8_t frame_index = index + HEADER_LEN;
				uint16_t checksum = _buf[GETINDEX(frame_index + 1)] << 8 | _buf[GETINDEX(frame_index + 0)];
				uint16_t signature = _buf[GETINDEX(frame_index + 3)] << 8 | _buf[GETINDEX(frame_index + 2)];
				uint16_t pixel_x = _buf[GETINDEX(frame_index + 5)] << 8 | _buf[GETINDEX(frame_index + 4)];
				uint16_t pixel_y = _buf[GETINDEX(frame_index + 7)] << 8 | _buf[GETINDEX(frame_index + 6)];
				uint16_t pixel_size_x = _buf[GETINDEX(frame_index + 9)] << 8 | _buf[GETINDEX(frame_index + 8)];
				uint16_t pixel_size_y = _buf[GETINDEX(frame_index + 11)] << 8 | _buf[GETINDEX(frame_index + 10)];

				/** crc check **/
				if (signature + pixel_x + pixel_y + pixel_size_x + pixel_size_y != checksum) {
					_read_failures++;
					ret = false;
				} else {
					ret = true;
				}

				/** convert to angles **/
				block->signature = signature;
				block->pos_x = (pixel_x - IRLOCK_CENTER_X) * IRLOCK_TAN_ANG_PER_PIXEL_X;
				block->pos_y = (pixel_y - IRLOCK_CENTER_Y) * IRLOCK_TAN_ANG_PER_PIXEL_Y;
				block->size_x = pixel_size_x * IRLOCK_TAN_ANG_PER_PIXEL_X;
				block->size_y = pixel_size_y * IRLOCK_TAN_ANG_PER_PIXEL_Y;

				// set the tail to one after the frame because we neglect
				// any data before the one we just read
				_tail = GETINDEX(index + FRAME_LEN);
				break;
			}

			no_header_counter = 0;

		} else {
			no_header_counter++;
		}

		index--;

		if (index < 0) {
			index = BUF_LEN - 1;
		}
	}

	return ret;
}

void
IRLOCK::read_device()
{
	int fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		PX4_WARN("serial port not open");
	}

	if (!isatty(fd)) {
		PX4_WARN("not a serial device");
	}


	// we poll on data from the serial port
	pollfd fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	// read buffer
	uint8_t buf[BUF_LEN];

	while (!_task_should_exit) {
		// wait for up to 20ms for data
		int pret = ::poll(fds, (sizeof(fds) / sizeof(fds[0])), 20);

		// timed out
		if (pret == 0) {
			continue;
		}

		if (pret < 0) {
			PX4_DEBUG("IRLOCK serial port poll error");
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			memset(&buf[0], 0, sizeof(buf));
			int len = ::read(fd, &buf[0], sizeof(buf));

			if (len <= 0) {
				PX4_DEBUG("error reading IRLOCK");
			}

			struct irlock_s report;

			report.timestamp = hrt_absolute_time();

			report.num_targets = 0;

			while (report.num_targets < IRLOCK_OBJECTS_MAX) {
				if (!read_and_parse(&buf[0], len, &report.targets[report.num_targets])) {
					break;
				}

				report.num_targets++;
			}

			_reports->force(&report);

			// publish over uORB
			if (report.num_targets > 0) {
				struct irlock_report_s orb_report;

				orb_report.timestamp = report.timestamp;
				orb_report.signature = report.targets[0].signature;
				orb_report.pos_x     = report.targets[0].pos_x;
				orb_report.pos_y     = report.targets[0].pos_y;
				orb_report.size_x    = report.targets[0].size_x;
				orb_report.size_y    = report.targets[0].size_y;

				if (_irlock_report_topic != nullptr) {
					orb_publish(ORB_ID(irlock_report), _irlock_report_topic, &orb_report);

				} else {
					_irlock_report_topic = orb_advertise_multi(ORB_ID(irlock_report), &orb_report, &_orb_class_instance, ORB_PRIO_LOW);

					if (_irlock_report_topic == nullptr) {
						DEVICE_LOG("failed to create irlock_report object. Did you start uOrb?");
					}
				}
			}
		}
	}

	::close(fd);
}

void irlock_usage()
{
	warnx("missing command: try 'start', 'stop', 'info', 'test'");
	warnx("options:");
	warnx("    usart port(%d)", IRLOCK_DEFAULT_PORT);
}

int irlock_main(int argc, char *argv[])
{
	/** jump over start/off/etc and look at options first **/
	if (getopt(argc, argv, "b:") != EOF) {
	}

	if (optind >= argc) {
		irlock_usage();
		exit(1);
	}

	const char *command = argv[optind];

	/** start driver **/
	if (!strcmp(command, "start")) {
		if (g_irlock != nullptr) {
			errx(1, "driver has already been started");
		}

		/** instantiate global instance **/
		if (argc > optind + 1) {
			g_irlock = new IRLOCK(argv[optind + 1]);

		} else {
			g_irlock = new IRLOCK(IRLOCK_DEFAULT_PORT);
		}

		if (g_irlock == nullptr) {
			errx(1, "failed to allocated memory for driver");
		}

		/** initialise global instance **/
		if (g_irlock->init() != OK) {
			IRLOCK *tmp_irlock = g_irlock;
			g_irlock = nullptr;
			delete tmp_irlock;
			errx(1, "failed to initialize device, stopping driver");
		}

		exit(0);
	}

	/** need the driver past this point **/
	if (g_irlock == nullptr) {
		warnx("not started");
		irlock_usage();
		exit(1);
	}

	/** stop the driver **/
	if (!strcmp(command, "stop")) {
		IRLOCK *tmp_irlock = g_irlock;
		g_irlock = nullptr;
		delete tmp_irlock;
		warnx("irlock stopped");
		exit(OK);
	}

	/** Print driver information **/
	if (!strcmp(command, "info")) {
		g_irlock->info();
		exit(OK);
	}

	/** test driver **/
	if (!strcmp(command, "test")) {
		g_irlock->test();
		exit(OK);
	}

	/** display usage info **/
	irlock_usage();
	exit(0);
}
