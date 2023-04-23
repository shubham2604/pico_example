cmake_minimum_required(VERSION 3.10)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
pico_sdk_init()


project(hello_world C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


file(GLOB C_SRC
       ${PROJECT_SOURCE_DIR}/src/*.c)

file(GLOB CPP_SRC
       ${PROJECT_SOURCE_DIR}/src/*.cpp)


add_executable(${PROJECT_NAME} ${C_SRC} ${CPP_SRC})



target_link_libraries(${PROJECT_NAME} pico_stdlib)

pico_add_extra_outputs(${PROJECT_NAME})
pico_enable_stdio_usb(${PROJECT_NAME} 1)

