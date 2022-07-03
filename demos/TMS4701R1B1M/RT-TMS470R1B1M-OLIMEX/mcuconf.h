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

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * TMS470R1B1M drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the driver
 * is enabled in halconf.h.
 */

/*
 * HAL driver system settings.
 */
#define TMS470R1B1M_OSC_HZ      (4000000)
#define TMS470R1B1M_CPU_HZ      (32000000)
#define TMS470R1B1M_PERIPH_HZ   (32000000)

/*
 * ADC driver system settings.
 */

/*
 * CAN driver system settings.
 */

/*
 * MAC driver system settings.
 */

/*
 * PWM driver system settings.
 */

/*
 * SERIAL driver system settings.
 */
#define USE_TMS470R1B1M_UART0           TRUE
#define USE_TMS470R1B1M_UART1           FALSE
#define USE_TMS470R1B1M_UART2           FALSE

#endif /* MCUCONF_H */
