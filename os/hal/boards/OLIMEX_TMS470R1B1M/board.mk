# List of all the mandatory board related files.
BOARDSRC = ${CHIBIOS}/os/hal/boards/OLIMEX_TMS470R1B1M/board.c

# Required include directories
BOARDINC = ${CHIBIOS}/os/hal/boards/OLIMEX_TMS470R1B1M

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
