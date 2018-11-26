# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(MAIN_SRCS main/app.c main/blink.c main/wifi_tcp_server_sta.c main/ov7670.c )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(app)
