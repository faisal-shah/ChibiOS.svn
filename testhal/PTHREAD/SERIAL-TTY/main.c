/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdlib.h>
#include <stdio.h>
#include <libgen.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#define SHELL_WA_SIZE       THD_WORKING_AREA_SIZE(4096)
#define CONSOLE_WA_SIZE     THD_WORKING_AREA_SIZE(4096)
#define TEST_WA_SIZE        THD_WORKING_AREA_SIZE(4096)

#define cputs(msg) chMsgSend(cdtp, (msg_t)msg)

static thread_t *cdtp;
static thread_t *shelltp1;

static const ShellCommand commands[] = {
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SD1,
  commands
};

/*
 * Console print server done using synchronous messages. This makes the access
 * to the C printf() thread safe and the print operation atomic among threads.
 * In this example the message is the zero terminated string itself.
 */
static THD_FUNCTION(console_thread, arg) {

  (void)arg;
  while (!chThdShouldTerminateX()) {
    thread_t *tp = chMsgWait();
    puts((char *)chMsgGet(tp));
    fflush(stdout);
    chMsgRelease(tp, MSG_OK);
  }
}

static void start_shell_service(void) {

  shelltp1 = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
          "shell1", NORMALPRIO + 10,
          shellThread, (void *)&shell_cfg1);
  cputs("Shell service started on SD1");
}

/**
 * @brief Shell termination handler.
 *
 * @param[in] id event id.
 */
static void termination_handler(eventid_t id) {

  (void)id;
  if (shelltp1 && chThdTerminatedX(shelltp1)) {
    chThdWait(shelltp1);
    shelltp1 = NULL;
    chThdSleepMilliseconds(10);
    cputs("Init: shell on SD1 terminated");
    chSysLock();
    oqResetI(&SD1.oqueue);
    chSchRescheduleS();
    chSysUnlock();
    start_shell_service();
  }
}

static evhandler_t fhandlers[] = {
  termination_handler,
};

static SerialConfig sdconfig = {
#if USE_SIM_SERIAL_TTY == TRUE
  .setup_termios = false,
#endif
};

static void print_usage(char * prg)
{
  fprintf(stderr, "\n Usage: %s <serial device>\n", prg);
  fprintf(stderr, "\n Creating PTY pair:\n");
  fprintf(stderr, " socat -d -d pty,raw,echo=0 pty,raw,echo=0\n");
  fprintf(stderr, "\n Examples:\n");
  fprintf(stderr, " %s /dev/pts/3\n\n", prg);
  exit(0);
}

void process_cmd_line(int argc, char * argv[])
{
  if (argc != 2) {
    print_usage(basename(argv[0]));
  }
  sdconfig.dev_node = argv[1];
}

/*------------------------------------------------------------------------*
 * Simulator main.                                                        *
 *------------------------------------------------------------------------*/
int main(int argc, char * argv[])
{
  process_cmd_line(argc, argv);

  event_listener_t tel;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Serial ports (simulated) initialization.
   */
  sdStart(&SD1, &sdconfig);

  /*
   * Shell manager initialization.
   */
  shellInit();
  chEvtRegister(&shell_terminated, &tel, 0);

  /*
   * Console thread started.
   */
  cdtp = chThdCreateFromHeap(NULL, CONSOLE_WA_SIZE, "console",
                             NORMALPRIO + 1, console_thread, NULL);

  start_shell_service();

  /*
   * Events servicing loop.
   */
  while (!chThdShouldTerminateX())
    chEvtDispatch(fhandlers, chEvtWaitOne(ALL_EVENTS));

  return 0;
}
