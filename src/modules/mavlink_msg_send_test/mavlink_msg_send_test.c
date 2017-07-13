/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file mavlink_msg_send_test.c
 * mavlink_msg_send application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <uORB/uORB.h>
#include <uORB/topics/parafoil_att.h>
#include <uORB/topics/parafoil_attrate.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit = false;		/**< mavlink_msg_send exit flag */
static bool thread_running = false;			/**< mavlink_msg_send status flag */
static int mavlink_msg_send_task;			/**< Handle of mavlink_msg_send task / thread */

/**
 * mavlink_msg_send management function.
 */
__EXPORT int mavlink_msg_send_test_main(int argc, char *argv[]);

/**
 * Mainloop of mavlink_msg_send.
 */
int mavlink_msg_send_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: mavlink_msg_send {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The mavlink_msg_send app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mavlink_msg_send_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("mavlink_msg_send already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		mavlink_msg_send_task = px4_task_spawn_cmd("mavlink_msg_send",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 mavlink_msg_send_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int mavlink_msg_send_thread_main(int argc, char *argv[])
{

	warnx("[mavlink_msg_send] starting\n");

	//
	thread_running = true;
	struct parafoil_att_s parafoil_att_data;
	memset(&parafoil_att_data , 0, sizeof(parafoil_att_data));
	orb_advert_t parafoil_att_pub = orb_advertise(ORB_ID(parafoil_att), &parafoil_att_data);

	struct parafoil_attrate_s parafoil_attrate_data;
	memset(&parafoil_attrate_data , 0, sizeof(parafoil_attrate_data));
	orb_advert_t parafoil_attrate_pub = orb_advertise(ORB_ID(parafoil_attrate), &parafoil_attrate_data);

	while (!thread_should_exit) {

		parafoil_att_data.roll_angle = 1;
		parafoil_att_data.pitch_angle  = 1;
		parafoil_att_data.yaw_angle  = 1;
		orb_publish(ORB_ID(parafoil_att), parafoil_att_pub, &parafoil_att_data);
		PX4_WARN("publishing parafoil att: %5.3f %5.3f %5.3f",
				(double)parafoil_att_data.roll_angle,
				(double)parafoil_att_data.pitch_angle,
				(double)parafoil_att_data.yaw_angle);

		parafoil_attrate_data.roll_rate = 1.0f;
		parafoil_attrate_data.pitch_rate = 2.0f;
		parafoil_attrate_data.yaw_rate = 3.0f;
		orb_publish(ORB_ID(parafoil_attrate), parafoil_attrate_pub, &parafoil_attrate_data);
		PX4_WARN("publishing parafoil attrate: %5.3f %5.3f %5.3f",
				(double)parafoil_attrate_data.roll_rate,
				(double)parafoil_attrate_data.pitch_rate,
				(double)parafoil_attrate_data.yaw_rate);



		usleep(200);	/* sleep 200 us. -libn */
	}

	warnx("[mavlink_msg_send] exiting.\n");

	thread_running = false;

	return 0;
}
