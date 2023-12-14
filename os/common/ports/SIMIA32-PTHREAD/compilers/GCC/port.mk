# List of the ChibiOS/RT SIMIA32-PTHREAD port files.
PORTSRC = ${CHIBIOS}/os/common/ports/SIMIA32-PTHREAD/chcore.c

PORTASM =

PORTINC = $(CHIBIOS)/os/common/portability/GCC \
          ${CHIBIOS}/os/common/ports/SIMIA32-PTHREAD/compilers/GCC \
          ${CHIBIOS}/os/common/ports/SIMIA32-PTHREAD

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
