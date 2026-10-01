cmake_minimum_required(VERSION 3.13)

set(PICO_PLATFORM rp2350-arm-s)
set(PICO_BOARD vz80)
list(APPEND PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/src/boards)
if(NOT PICO_SDK_PATH)
  set(PICO_SDK_PATH $ENV{HOME}/pico-sdk)
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(vz80 C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

file(READ ${CMAKE_CURRENT_LIST_DIR}/../../VERSION VZ80_VERSION)
string(STRIP "${VZ80_VERSION}" VZ80_VERSION)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_LIST_DIR}/../../VERSION)

add_executable(vz80
  src/main.c
  src/crash.c
  src/flashfar.c
  src/vzdisk.c
  src/console.c
  src/led.c
  src/psram.c
  src/diag.c
  src/esp.c
  src/link.c
  src/bus.c
  src/z80.c
  src/z80_entry.c
  src/flashxip.c
  src/shelf.c
  src/cart.c
  src/fdc.c
  src/msx.c
  src/vdp.c src/vdpcmd.c
  src/kbd.c
  src/fwup.c
  src/settings.c
  src/slot.c
  src/audio.c
)
# The Z80 core runs on a local copy of its state; let GCC scalarise that struct into registers.
pico_generate_pio_header(vz80 ${CMAKE_CURRENT_LIST_DIR}/src/ws2812.pio)
pico_generate_pio_header(vz80 ${CMAKE_CURRENT_LIST_DIR}/src/bus.pio)
pico_generate_pio_header(vz80 ${CMAKE_CURRENT_LIST_DIR}/src/spi_slave.pio)
pico_generate_pio_header(vz80 ${CMAKE_CURRENT_LIST_DIR}/src/pwm.pio)
target_include_directories(vz80 PRIVATE src)
target_compile_definitions(vz80 PRIVATE
  VZ80_VERSION="${VZ80_VERSION}"
  PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
)
target_compile_options(vz80 PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(vz80
  pico_stdlib pico_unique_id pico_multicore
  hardware_pio hardware_dma hardware_flash hardware_xip_cache hardware_clocks hardware_adc hardware_uart
)
pico_set_binary_type(vz80 copy_to_ram)
pico_enable_stdio_usb(vz80 1)
pico_enable_stdio_uart(vz80 0)
pico_add_extra_outputs(vz80)
