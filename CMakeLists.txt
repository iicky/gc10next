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

pico_add_extra_outputs(main)

target_link_libraries(main
	pico_stdlib hardware_adc hardware_pwm hardware_i2c hardware_pio hardware_dma)
