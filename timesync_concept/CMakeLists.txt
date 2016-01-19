cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME timesync_concept)
project(${PROJECT_NAME})

# workaround for arduino mega board if fuses are set to use internal 8MHz os
set(F_CPU_WORKAROUND 8000000)

set(${CMAKE_PROJECT_NAME}_BOARD mega)
set(${CMAKE_PROJECT_NAME}_SKETCH timesync_concept.ino)
set(mega.build.mcu atmega2560)


set(DEV_PORT_NUMER 1)

set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyACM${DEV_PORT_NUMER})
set(mega.upload.protocol wiring)
#set(mega.upload.speed 115200) # if running external 16MHz
set(mega.upload.speed 57600) # if running interal 8MHz osc

# settings for wrinting fuses only using external programmer:
#set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyUSB${DEV_PORT_NUMER})
#set(mega.upload.protocol stk500)
#set(mega.upload.speed 19200)

generate_arduino_firmware(${CMAKE_PROJECT_NAME})
