cmake_minimum_required(VERSION 3.10)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init()


project(pico_tutorials C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)
##############################################################

file(GLOB C_HELLOWORLD_SRC
       ${PROJECT_SOURCE_DIR}/src/hello_world/*.c)

file(GLOB CPP_HELLOWORLD_SRC
       ${PROJECT_SOURCE_DIR}/src/hello_world/*.cpp)


add_executable(hello_world ${C_HELLOWORLD_SRC} ${CPP_HELLOWORLD_SRC})

target_link_libraries(hello_world pico_stdlib)
###############################################################

file(GLOB C_I2CSCAN_SRC
       ${PROJECT_SOURCE_DIR}/src/i2c/*.c)

file(GLOB CPP_I2CSCAN_SRC
       ${PROJECT_SOURCE_DIR}/src/i2c/*.cpp)


add_executable(i2c_scan ${C_I2CSCAN_SRC} ${CPP_I2CSCAN_SRC})

target_link_libraries(i2c_scan pico_stdlib hardware_i2c)


##############################################################

file(GLOB C_I2C_MPU6050_SRC
       ${PROJECT_SOURCE_DIR}/src/mpu6050/*.c)

file(GLOB CPP_I2C_MPU6050_SRC
       ${PROJECT_SOURCE_DIR}/src/mpu6050/*.cpp)


include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(i2c_mpu6050_read ${C_I2C_MPU6050_SRC} ${CPP_I2C_MPU6050_SRC})

target_include_directories(i2c_mpu6050_read PRIVATE ${PROJECT_SOURCE_DIR}/third_party/eigen-3.4.0)
target_link_libraries(i2c_mpu6050_read pico_stdlib hardware_i2c)
################################################################

pico_add_extra_outputs(hello_world)
pico_enable_stdio_usb(hello_world 1)

pico_add_extra_outputs(i2c_scan)
pico_enable_stdio_usb(i2c_scan 1)

pico_add_extra_outputs(i2c_mpu6050_read)
pico_enable_stdio_usb(i2c_mpu6050_read 1)

