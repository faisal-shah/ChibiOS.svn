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

/**
 * @file    simulator/posix/hal_serial_lld.c
 * @brief   Posix simulator low level serial driver code.
 *
 * @addtogroup POSIX_SERIAL
 * @{
 */

#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "hal.h"

#if USE_SIM_SERIAL_TTY == TRUE || defined(__DOXYGEN__)
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/stat.h>
#endif


#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief Serial driver 1 identifier.*/
#if USE_SIM_SERIAL1 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief Serial driver 2 identifier.*/
#if USE_SIM_SERIAL2 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief Driver default configuration.*/
#if USE_SIM_SERIAL_TTY == FALSE || defined(__DOXYGEN__)
static const SerialConfig default_config = {
};
#endif

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#if USE_SIM_SERIAL_TTY == TRUE
#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})

static int init(SerialDriver *sdp, const SerialConfig *config) {
  osalDbgCheck(sdp != NULL && config != NULL);

  struct termios tc_cfg;

  if (tcgetattr(sdp->fd, &tc_cfg) < 0) {
    perror("hal_serial_lld::init: Failed to get attr of device");
    return -1;
  }

  tc_cfg.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | ISTRIP);

  switch (config->parity) {
    case 0:
      tc_cfg.c_iflag &= ~(IGNPAR | PARMRK | INPCK);
      break;
    case 1:
      tc_cfg.c_iflag &= ~(IGNPAR);
      tc_cfg.c_iflag |= (PARMRK | INPCK);
      tc_cfg.c_cflag |= PARENB;
      tc_cfg.c_cflag |= PARODD;
      break;
    case 2:
      tc_cfg.c_iflag &= ~(IGNPAR);
      tc_cfg.c_iflag |= (PARMRK | INPCK);
      tc_cfg.c_cflag |= PARENB;
      tc_cfg.c_cflag &= ~(PARODD);
      break;
    default:
      fprintf(stderr, "Invalid parity selection");
      return -1;
  }

  switch(config->flow_control) {
    case 0:
      tc_cfg.c_iflag &= ~(IXON | IXOFF);
      break;
    case 1:
      tc_cfg.c_iflag |= (IXON | IXOFF);
      break;
    default:
      fprintf(stderr, "Invalid flow_control selection");
      return -1;
  }

  tc_cfg.c_oflag = 0;

  tc_cfg.c_cflag &= ~(CSIZE);
  switch(config->data_size) {
    case 7:
      tc_cfg.c_cflag |= CS7;
      break;
    case 8:
      tc_cfg.c_cflag |= CS8;
      break;
    default:
      fprintf(stderr, "Invalid data_size selection");
      return -1;
  }

  switch(config->stop_bit) {
    case 1:
      tc_cfg.c_cflag &= ~(CSTOPB);
      break;
    case 2:
      tc_cfg.c_cflag |= CSTOPB;
      break;
    default:
      fprintf(stderr, "Invalid stop bit selection");
      return -1;
  }

  tc_cfg.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  tc_cfg.c_cc[VMIN]  = 1;
  tc_cfg.c_cc[VTIME] = 0;

  if (cfsetspeed(&tc_cfg, config->speed) < 0) {
    fprintf(stderr, "Failed to set serial device speed");
    return -1;
  }

  if (tcsetattr(sdp->fd, TCSAFLUSH, &tc_cfg) < 0) {
    fprintf(stderr, "Failed to set serial device attributes");
    return -1;
  }

  return 0;
}

static void onotify_cb(io_queue_t * qp) {
  SerialDriver * sdp = container_of(qp, SerialDriver, oqueue);

  uint8_t data[1];
  int n = 0;

  while((n = sdRequestDataI(sdp)) != MSG_TIMEOUT) {
    data[0] = n;
    n = write(sdp->fd, data, sizeof(data));
    switch(n) {
      case 0:
        fprintf(stderr, "data not written to tty device\n");
        goto cleanup;
      case -1:
        if (errno != EAGAIN)
          perror("outint: read on tty device failed");
        goto cleanup;
    }
  } while(n != MSG_TIMEOUT);

cleanup:
  chSchRescheduleS();
}

static void serve_interrupt(SerialDriver * sdp) {
  if ((sdp == NULL) || (sdp->fd == -1))
    return;

  uint8_t data[SERIAL_BUFFERS_SIZE];

  while (1) {
    int n = read(sdp->fd, data, sizeof(data));
    switch(n) {
      case 0:
        return;
      case -1:
        if (errno != EAGAIN)
          perror("inint: read on tty device failed");
        return;
    }

    for (int i = 0; i < n; i++) {
      osalSysLockFromISR();
      if (iqIsEmptyI(&sdp->iqueue))
        chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
      if (iqPutI(&sdp->iqueue, data[i]) < MSG_OK) {
        chnAddFlagsI(sdp, SD_QUEUE_FULL_ERROR);
        osalSysUnlockFromISR();
        return;
      }
      osalSysUnlockFromISR();
    }
  }
}

static bool inint(SerialDriver * sdp) {

  if ((sdp == NULL) || (sdp->fd == -1))
    return false;

  fd_set fd_in;
  FD_ZERO(&fd_in);
  FD_SET(sdp->fd, &fd_in);
  struct timeval tv = {0};

  int ret = select(sdp->fd + 1, &fd_in, NULL, NULL, &tv);
  switch(ret)
  {
    case -1:
      perror("inint: select() call on tty device failed.");
      return false;
    case 0:
      return false;
  }
  if (FD_ISSET(sdp->fd, &fd_in))
    return true;

  return false;
}

#else

static void init(SerialDriver *sdp, uint16_t port) {
  struct sockaddr_in sad;
  struct protoent *prtp;
  int sockval = 1;
  socklen_t socklen = sizeof(sockval);

  if ((prtp = getprotobyname("tcp")) == NULL) {
    printf("%s: Error mapping protocol name to protocol number\n", sdp->com_name);
    goto abort;
  }

  sdp->com_listen = socket(PF_INET, SOCK_STREAM, prtp->p_proto);
  if (sdp->com_listen == -1) {
    printf("%s: Error creating simulator socket\n", sdp->com_name);
    goto abort;
  }

  setsockopt(sdp->com_listen, SOL_SOCKET, SO_REUSEADDR, &sockval, socklen);

#if 0
  if (ioctl(sdp->com_listen, FIONBIO, &nb) != 0) {
    printf("%s: Unable to setup non blocking mode on socket\n", sdp->com_name);
    goto abort;
  }
#endif
  int flags = fcntl(sdp->com_listen, F_GETFL, 0);
  if (fcntl(sdp->com_listen, F_SETFL, flags | O_NONBLOCK) != 0) {
    printf("%s: Unable to setup non blocking mode on socket\n", sdp->com_name);
    goto abort;
  }

  memset(&sad, 0, sizeof(sad));
  sad.sin_family = AF_INET;
  sad.sin_addr.s_addr = INADDR_ANY;
  sad.sin_port = htons(port);
  if (bind(sdp->com_listen, (struct sockaddr *)&sad, sizeof(sad))) {
    printf("%s: Error binding socket\n", sdp->com_name);
    goto abort;
  }

  if (listen(sdp->com_listen, 1) != 0) {
    printf("%s: Error listening socket\n", sdp->com_name);
    goto abort;
  }
  printf("Full Duplex Channel %s listening on port %d\n", sdp->com_name, port);
  return;

abort:
  if (sdp->com_listen != -1)
    close(sdp->com_listen);
  exit(1);
}

static bool connint(SerialDriver *sdp) {

  if (sdp->com_data == -1) {
    struct sockaddr addr;
    socklen_t addrlen = sizeof(addr);

    if ((sdp->com_data = accept(sdp->com_listen, &addr, &addrlen)) == -1)
      return false;

#if 0
    if (ioctl(sdp->com_data, FIONBIO, &nb) != 0) {
      printf("%s: Unable to setup non blocking mode on data socket\n", sdp->com_name);
      goto abort;
    }
#endif
    int flags = fcntl(sdp->com_data, F_GETFL, 0);
    if (fcntl(sdp->com_data, F_SETFL, flags | O_NONBLOCK) != 0) {
      printf("%s: Unable to setup non blocking mode on data socket\n", sdp->com_name);
      goto abort;
    }

    osalSysLockFromISR();
    chnAddFlagsI(sdp, CHN_CONNECTED);
    osalSysUnlockFromISR();
    return true;
  }
  return false;
abort:
  if (sdp->com_listen != -1)
    close(sdp->com_listen);
  if (sdp->com_data != -1)
    close(sdp->com_data);
  exit(1);
}

static bool inint(SerialDriver *sdp) {

  if (sdp->com_data != -1) {
    int i;
    uint8_t data[32];

    /*
     * Input.
     */
    int n = recv(sdp->com_data, data, sizeof(data), 0);
    switch (n) {
    case 0:
      close(sdp->com_data);
      sdp->com_data = -1;
      osalSysLockFromISR();
      chnAddFlagsI(sdp, CHN_DISCONNECTED);
      osalSysUnlockFromISR();
      return false;
    case -1:
      if (errno == EWOULDBLOCK)
        return false;
      close(sdp->com_data);
      sdp->com_data = -1;
      return false;
    }
    for (i = 0; i < n; i++) {
      osalSysLockFromISR();
      sdIncomingDataI(sdp, data[i]);
      osalSysUnlockFromISR();
    }
    return true;
  }
  return false;
}

static bool outint(SerialDriver *sdp) {

  if (sdp->com_data != -1) {
    int n;
    uint8_t data[1];

    /*
     * Input.
     */
    osalSysLockFromISR();
    n = sdRequestDataI(sdp);
    osalSysUnlockFromISR();
    if (n < 0)
      return false;
    data[0] = (uint8_t)n;
    n = send(sdp->com_data, data, sizeof(data), 0);
    switch (n) {
    case 0:
      close(sdp->com_data);
      sdp->com_data = -1;
      osalSysLockFromISR();
      chnAddFlagsI(sdp, CHN_DISCONNECTED);
      osalSysUnlockFromISR();
      return false;
    case -1:
      if (errno == EWOULDBLOCK)
        return false;
      close(sdp->com_data);
      sdp->com_data = -1;
      return false;
    }
    return true;
  }
  return false;
}

#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * Low level serial driver initialization.
 */
void sd_lld_init(void) {

#if USE_SIM_SERIAL1
#if USE_SIM_SERIAL_TTY == TRUE
  sdObjectInit(&SD1, NULL, onotify_cb);
  SD1.dev_node = NULL;
  SD1.fd = -1;
#else
  sdObjectInit(&SD1, NULL, NULL);
  SD1.com_listen = -1;
  SD1.com_data = -1;
  SD1.com_name = "SD1";
#endif
#endif

#if USE_SIM_SERIAL2
#if USE_SIM_SERIAL_TTY == TRUE
  sdObjectInit(&SD2, NULL, onotify_cb);
  SD2.dev_node = NULL;
  SD2.fd = -1;
#else
  sdObjectInit(&SD2, NULL, NULL);
  SD2.com_listen = -1;
  SD2.com_data = -1;
  SD2.com_name = "SD2";
#endif
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

#if USE_SIM_SERIAL_TTY == TRUE
  osalDbgAssert((config != NULL) && (config->dev_node != NULL),
  "config and device node path must be provided for tty device");

  sdp->dev_node = config->dev_node;

  sdp->fd = open(sdp->dev_node, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (sdp->fd == -1) {
    perror("sd_lld_start: Failed to open tty device: ");
    return;
  }

  if (!isatty(sdp->fd)) {
    perror("sd_lld_start: Invalid device, check SerialConfig::device_node");
    return;
  }

#if USE_SIM_SERIAL1
  if ((sdp == &SD1) && (config->setup_termios))
    if (init(&SD1, config) < 0)
      return;
#endif

#if USE_SIM_SERIAL2
  if ((sdp == &SD2) && (config->setup_termios))
    if (init(&SD2, config) < 0)
      return;
#endif

  chnAddFlagsI(sdp, CHN_CONNECTED);

#else
  if (config == NULL)
    config = &default_config;

#if USE_SIM_SERIAL1
  if (sdp == &SD1)
    init(&SD1, SIM_SD1_PORT);
#endif

#if USE_SIM_SERIAL2
  if (sdp == &SD2)
    init(&SD2, SIM_SD2_PORT);
#endif
#endif
}

/**
 * @brief Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp pointer to a @p SerialDriver object
 */
void sd_lld_stop(SerialDriver *sdp) {

#if USE_SIM_SERIAL_TTY == TRUE
  osalDbgCheck(sdp != NULL);
  int ret = close(sdp->fd);
  if (ret != 0)
  {
    perror("sd_lld_stop: Failed to close tty device");
    return;
  }
#else
  (void)sdp;
#endif
}

#if USE_SIM_SERIAL_TTY == TRUE
#if USE_SIM_SERIAL1
void sd_lld_irq_handler_serial1(void)
{
  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif
#if USE_SIM_SERIAL2
void sd_lld_irq_handler_serial2(void)
{
  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif
bool sd_lld_interrupt_pending(SerialDriver * sdp)
{
  bool b;

  b = inint(sdp);

  return b;
}
#else
bool sd_lld_interrupt_pending(void) {
  bool b;

  OSAL_IRQ_PROLOGUE();

  b =  connint(&SD1) || connint(&SD2) ||
       inint(&SD1)   || inint(&SD2)   ||
       outint(&SD1)  || outint(&SD2);

  OSAL_IRQ_EPILOGUE();

  return b;
}
#endif

#endif /* HAL_USE_SERIAL */

/** @} */
