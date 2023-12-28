*****************************************************************************
** ChibiOS/RT port for x86 into a Posix process                            **
*****************************************************************************

** TARGET **

The demo runs under any Posix IA32 system as an application program.

** The Demo **

A GPT is used to trigger the ADC conversions.

This demonstrates the usage of the simulator GPT, and ADC.

The simulator GPT has a secondary callback function, which can serve to
trigger other peripherals behind the scenes - as you would expect a timer
peripheral on an MCU to do. In this demo, it is triggering the ADC conversions
every 100ms.

The simulator ADC has a few unique features:
  - A low level function `sim_adc_lld_trigger_conversion`, which can be used to
    manually trigger a conversion. It will return non-zero if a conversion is
    currently in process - otherwise 0.
  - A `channels_default` field in the ADCConversionGroup which serves as the
    initial ADC values for each channel.
  - A `start_conv_cb` which is called on the start of a conversion. Here you
    can prepare ADC channel data to be simulated.
  - A `conv_cb` which is called when the conversion is finished. Here you can
    copy the simulated channel data into ADCDriver::channel_data

** Build Procedure **

The demo was built using GCC.

** Sample output **

ChibiOS/RT simulator (Linux)

triggered = 0
Samples: 1 2 3 4 5 6 7 8 9 10 1 2 3 4 5 6 7 8 9 10
triggered = 0
Samples: 2 3 4 5 6 7 8 9 10 11 2 3 4 5 6 7 8 9 10 11
triggered = 0
Samples: 3 4 5 6 7 8 9 10 11 12 3 4 5 6 7 8 9 10 11 12
triggered = 0
Samples: 4 5 6 7 8 9 10 11 12 13 4 5 6 7 8 9 10 11 12 13
triggered = 0
Samples: 5 6 7 8 9 10 11 12 13 14 5 6 7 8 9 10 11 12 13 14
triggered = 0
Samples: 6 7 8 9 10 11 12 13 14 15 6 7 8 9 10 11 12 13 14 15
triggered = 0
Samples: 7 8 9 10 11 12 13 14 15 16 7 8 9 10 11 12 13 14 15 16
triggered = 0
Samples: 8 9 10 11 12 13 14 15 16 17 8 9 10 11 12 13 14 15 16 17
triggered = 0
Samples: 9 10 11 12 13 14 15 16 17 18 9 10 11 12 13 14 15 16 17 18
triggered = 0
Samples: 10 11 12 13 14 15 16 17 18 19 10 11 12 13 14 15 16 17 18 19
