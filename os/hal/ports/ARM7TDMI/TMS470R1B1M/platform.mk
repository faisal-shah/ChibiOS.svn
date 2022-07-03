# List of all the LPC214x platform files.
PLATFORMSRC = ${CHIBIOS}/os/hal/ports/ARM7TDMI/TMS470R1B1M/hal_lld.c \
              ${CHIBIOS}/os/hal/ports/ARM7TDMI/TMS470R1B1M/hal_st_lld.c \
              ${CHIBIOS}/os/hal/ports/ARM7TDMI/TMS470R1B1M/hal_serial_lld.c \
              ${CHIBIOS}/os/hal/ports/ARM7TDMI/TMS470R1B1M/nonvic.c

# Required include directories
PLATFORMINC = ${CHIBIOS}/os/hal/ports/ARM7TDMI/TMS470R1B1M

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
