cmake_minimum_required(VERSION 3.13)

include (pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# The original build enabled no warnings at all, which is how a printf with a
# %s and no argument survived in the EEPROM error path. Must precede
# add_executable() to apply to the target.
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

# Commit id for the firmware version string. In a checkout it is read straight
# from git, so it is self-verifying. The Docker build has no .git and verifies
# nothing: it only forces the caller to supply an id rather than defaulting to
# "dev". Whether that id describes the copied source is the caller's
# responsibility, and a wrong value still yields a wrongly labelled image.
# A trailing '+' means the checkout did not match the commit exactly, whether
# from modified tracked files or new untracked ones, so the hash alone would
# not describe what is actually running. Ignored paths (build trees, a vendored
# SDK) are excluded, so they never trip it.
#
# An empty -DFW_BUILD_ID counts as unset: passing one through an undefined
# shell variable defines the cache entry as "", which would otherwise skip
# the derivation below and silently ship a bare "FWNX1E-".
if(NOT DEFINED FW_BUILD_ID OR FW_BUILD_ID STREQUAL "")
	find_package(Git QUIET)
	if(GIT_FOUND)
		execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short=7 HEAD
			WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
			OUTPUT_VARIABLE _fw_sha OUTPUT_STRIP_TRAILING_WHITESPACE
			ERROR_QUIET RESULT_VARIABLE _fw_rc)
		if(_fw_rc EQUAL 0)
			execute_process(COMMAND ${GIT_EXECUTABLE} status --porcelain
				WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
				OUTPUT_VARIABLE _fw_dirty OUTPUT_STRIP_TRAILING_WHITESPACE
				ERROR_QUIET)
			if(NOT _fw_dirty STREQUAL "")
				set(_fw_sha "${_fw_sha}+")
			endif()
			set(FW_BUILD_ID "${_fw_sha}")
		endif()
	endif()
endif()
if(NOT DEFINED FW_BUILD_ID OR FW_BUILD_ID STREQUAL "")
	set(FW_BUILD_ID "dev")
endif()
message(STATUS "Firmware build id: ${FW_BUILD_ID}")

add_executable(main
	main.c ssd1306.c
	)

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/pulse_ts.pio)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

# SDK 2.x made PICO_DEFAULT_UART_INSTANCE a function-like macro, so defining it
# to `uart1` broke every translation unit. Select the instance by index instead:
# this spelling works on both SDK 1.5.x and 2.x.
add_compile_definitions(PICO_DEFAULT_UART=1)
add_compile_definitions(PICO_DEFAULT_UART_TX_PIN=4)
add_compile_definitions(PICO_DEFAULT_UART_RX_PIN=5)

target_compile_definitions(main PRIVATE FW_BUILD_ID="${FW_BUILD_ID}")

pico_add_extra_outputs(main)

target_link_libraries(main
	pico_stdlib hardware_adc hardware_pwm hardware_i2c hardware_pio hardware_dma)
