##############################################################################
# Multi-project makefile rules
#

all:
	@echo
	@echo === Building for STM32L476-Discovery ===============================
	+@make --no-print-directory -f ./make/stm32l476_discovery.make all
	@echo ====================================================================
	@echo

	@echo
	@echo === Building for STM32F722-Nucleo-144 ==============================
	+@make --no-print-directory -f ./make/stm32f722_nucleo-144.make all
	@echo ====================================================================
	@echo

clean:
	@echo
	+@make --no-print-directory -f ./make/stm32l476_discovery.make clean
	@echo

	@echo
	+@make --no-print-directory -f ./make/stm32f722_nucleo-144.make clean
	@echo

#
##############################################################################
