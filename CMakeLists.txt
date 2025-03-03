cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS
    ON
    CACHE INTERNAL "")

set(PICO_SDK_FETCH_FROM_GIT on)
include(pico_sdk_import.cmake)
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES
      ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

project(Telemetrix4RpiPico)
pico_sdk_init()
add_executable(
  Telemetrix4RpiPico
  src/Telemetrix4RpiPico.cpp
  src/i2c_helpers.cpp
  src/uart.cpp
  src/serialization.cpp
  src/drivers/HMC5883L.cpp
  src/drivers/HX711.cpp
  src/drivers/HiwonderServo.cpp
  src/drivers/INA226v2.cpp
  src/drivers/VL53L0.cpp
  src/drivers/neopixel.cpp
  src/module/Hiwonder_Servo.cpp
  src/module/PCA9685_Module.cpp
  src/module/module.cpp
  src/module/tmx_ssd1306_Module.cpp
  src/sensors/adxl345_sensor.cpp
  src/sensors/gps_sensor.cpp
  src/sensors/hmc5883l_sensor.cpp
  src/sensors/hx711_sensor.cpp
  src/sensors/ina226_sensor.cpp
  src/sensors/mpu9250_sensor.cpp
  src/sensors/sensor.cpp
  src/sensors/veml6040_sensor.cpp
  src/sensors/vl53l0x_sensor.cpp)
find_package(Git REQUIRED)
execute_process(
  COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
  OUTPUT_VARIABLE COMMIT_ID
  OUTPUT_STRIP_TRAILING_WHITESPACE)

pico_set_program_version(Telemetrix4RpiPico "1.3_${COMMIT_ID}")

target_include_directories(Telemetrix4RpiPico
                           PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include/)
# target_compile_options(Telemetrix4RpiPico PRIVATE  -Wall -Wextra -Wpedantic
# -Wconversion -Wno-sign-conversion -Wdouble-promotion -Wno-unused-parameter
# -Wno-float-conversion -Wno-conversion -Wno-double-promotion
# -Wno-unused-function -Werror  -Wuninitialized)
pico_generate_pio_header(Telemetrix4RpiPico
                         ${CMAKE_CURRENT_LIST_DIR}/src/Telemetrix4RpiPico.pio)
pico_enable_stdio_usb(Telemetrix4RpiPico 1)
pico_enable_stdio_uart(Telemetrix4RpiPico 0)
pico_add_extra_outputs(Telemetrix4RpiPico)
add_subdirectory(pico-ssd1306)

target_link_libraries(
  Telemetrix4RpiPico
  pico_stdlib
  hardware_pwm
  pico_unique_id
  hardware_watchdog
  hardware_adc
  hardware_i2c
  hardware_pio
  hardware_clocks
  hardware_dma
  hardware_spi
  hardware_flash
  pico_sync
  pico_ssd1306)

set(SKIP_BOOTLOADER_ENTRY_PIN 1)
add_subdirectory(rp2040-serial-bootloader)
# pico_set_linker_script(Telemetrix4RpiPico
# ${CMAKE_CURRENT_SOURCE_DIR}/blink_noboot2.ld)

bootloader_build_combined(Telemetrix4RpiPico)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(
  lint
  COMMAND
    sh -c
    "run-clang-tidy ${CMAKE_SOURCE_DIR}/src/Telemetrix4RpiPico.cpp -checks=`tr '\\n' , <${CMAKE_SOURCE_DIR}/checks.txt` >lint.out 2>lint.err"
  COMMAND sh -c "grep warning: lint.out || true"
  COMMAND ls -lh ${CMAKE_BINARY_DIR}/lint.out
  VERBATIM
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
