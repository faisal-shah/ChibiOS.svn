#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <libgen.h>
#include <time.h>
#include "ch.h"
#include "hal.h"

static CANConfig sCanConfig;

static void print_usage(char * prg)
{
  fprintf(stderr, "\n Usage: %s <can interface>\n", prg);
  fprintf(stderr, "\n Examples:\n");
  fprintf(stderr, " %s vcan0\n\n", prg);
  exit(0);
}

void process_cmd_line(int argc, char * argv[])
{
  if (argc != 2) {
    print_usage(basename(argv[0]));
  }
  sCanConfig.ifname = argv[1];
}

static struct timespec sStartTime;
static void can_rx_full_cb(CANDriver * canp, uint32_t flags)
{
  osalDbgCheck(canp != NULL);

  (void)flags;
  CANRxFrame raw_msg;
  struct timespec stamp, dt;
  clock_gettime(CLOCK_MONOTONIC, &stamp);
  dt.tv_sec = stamp.tv_sec - sStartTime.tv_sec;
  dt.tv_nsec = stamp.tv_nsec - sStartTime.tv_nsec;
  if (dt.tv_nsec < 0)
  {
      dt.tv_sec--;
      dt.tv_nsec += 1000000000;
  }
  if (dt.tv_sec < 0)
  {
      dt.tv_sec = 0;
      dt.tv_nsec = 0;
  }

  osalSysLockFromISR();
  while (!canTryReceiveI(canp, CAN_ANY_MAILBOX, &raw_msg))
  {
      printf("(%03llu.%06llu) ", (unsigned long long)dt.tv_sec, (unsigned long long)dt.tv_nsec/1000U);
      printf("%08X#", raw_msg.EID);
      if (raw_msg.DLC <= 8U)
      {
          for (size_t i = 0; i < raw_msg.DLC; i++)
          {
              printf("%02X", raw_msg.data8[i]);
          }
      }
      printf("\n");
  }

  osalSysUnlockFromISR();
}

int main(int argc, char * argv[])
{
  process_cmd_line(argc, argv);
  clock_gettime(CLOCK_MONOTONIC, &sStartTime);

  halInit();
  chSysInit();

  CAND1.rxfull_cb = can_rx_full_cb;
  CAND1.txempty_cb = NULL;
  CAND1.error_cb = NULL;
  canStart(&CAND1, (const CANConfig *)&sCanConfig);

  size_t ctr = 0;
  CANTxFrame txfrm;
  memset(&txfrm, 0, sizeof(txfrm));

  txfrm.DLC = 4;
  txfrm.RTR = 0;
  txfrm.IDE = 1;
  txfrm.EID = 0x80085;

  while (1)
  {
    chThdSleepMilliseconds(1000);
    txfrm.data32[0] = ctr++;
    canTransmitTimeout(&CAND1, 0, &txfrm, TIME_MS2I(20));
  }
}
