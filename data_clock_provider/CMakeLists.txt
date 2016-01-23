cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME data_clock_provider)
project(${PROJECT_NAME})

# optimize for speed
set(ARDUINO_C_FLAGS "-g -O3 -w -ffunction-sections -fdata-sections -MMD")
set(ARDUINO_CXX_FLAGS "-g -O3 -w -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -MMD")
set(ARDUINO_LINKER_FLAGS "-w -O3 -Wl,--gc-sections")

# arduino mega 2560 uses external 16000000 osc but this example needs 8000000

set(F_CPU_WORKAROUND 8000000)

set(${CMAKE_PROJECT_NAME}_BOARD mega)
set(${CMAKE_PROJECT_NAME}_SKETCH data_clock_provider.ino)
set(mega.build.mcu atmega2560)

# the port number i.e. /dev/ttyACM0 => port 0
SET(DEV_PORT_NUMER 0)

set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyACM${DEV_PORT_NUMER})
set(mega.upload.protocol wiring)
#set(mega.upload.speed 115200) # if running external 16MHz
set(mega.upload.speed 57600) # if running interal 8MHz osc

# settings for wrinting fuses only: using external programmer:
#set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyUSB${DEV_PORT_NUMER})
#set(mega.upload.protocol stk500)
#set(mega.upload.speed 19200)

generate_arduino_firmware(${CMAKE_PROJECT_NAME})
