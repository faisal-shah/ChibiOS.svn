##############################################################################
# Multi-project makefile rules
#

JOBS := $(shell nproc)

all:
	@echo
	@echo === Building for STM32L476-Discovery ===============================
	+@make --no-print-directory -f ./make/stm32l476_discovery.make all -j$(JOBS)
	@echo ====================================================================
	@echo

	@echo
	@echo === Building for STM32F722-Nucleo-144 ==============================
	+@make --no-print-directory -f ./make/stm32f722_nucleo-144.make all -j$(JOBS)
	@echo ====================================================================
	@echo

	@echo
	@echo === Building for STM32F767-Nucleo-144 ==============================
	+@make --no-print-directory -f ./make/stm32f767_nucleo-144.make all -j$(JOBS)
	@echo ====================================================================
	@echo

	@echo
	@echo === Building for STM32F746-Discovery ===============================
	+@make --no-print-directory -f ./make/stm32f746_discovery.make all -j$(JOBS)
	@echo ====================================================================
	@echo

	@echo
	@echo === Building for STM32L496-Nucleo-144 ==============================
	+@make --no-print-directory -f ./make/stm32l496_nucleo-144.make all -j$(JOBS)
	@echo ====================================================================
	@echo

clean:
	@echo
	+@make --no-print-directory -f ./make/stm32l476_discovery.make clean
	@echo

	@echo
	+@make --no-print-directory -f ./make/stm32f722_nucleo-144.make clean
	@echo

	@echo
	+@make --no-print-directory -f ./make/stm32f767_nucleo-144.make clean
	@echo

	@echo
	+@make --no-print-directory -f ./make/stm32f746_discovery.make clean
	@echo

	@echo
	+@make --no-print-directory -f ./make/stm32l496_nucleo-144.make clean
	@echo

#
##############################################################################
