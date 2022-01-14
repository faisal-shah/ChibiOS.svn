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

#include <stdio.h>
#include <stdbool.h>

#include "sbuser.h"

/*
 * Application entry point.
 */
int main(int argc, char *argv[], char *envp[]) {
  char *s;
  unsigned i = 1U;

  printf("argc: %d\r\n", argc);
  printf("argv: ");
  while ((s = *argv++) != NULL) {
    printf("%s", s);
  }
  printf("\r\n");
  printf("envp: ");
  while ((s = *envp++) != NULL) {
    printf("%s", s);
  }
  printf("\r\n");

  while (true) {
    printf("#1 Hello World (%u)!!\r\n", i++);
    sbSleepMilliseconds(500);
  }
}
