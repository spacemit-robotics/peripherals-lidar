# drv_uart_rplidar: fetch rplidar SDK and expose components::rplidar_sdk (B2: single definition)
if(TARGET components::rplidar_sdk)
  return()
endif()

get_filename_component(_LIDAR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${_LIDAR_ROOT}/cmake/FetchThirdParty.cmake")

# This driver uses the sl_* API (sl_lidar.h / sl_lidar_driver.h).
# Track upstream master latest.
set(_RPLIDAR_GIT_REPO "https://github.com/Slamtec/rplidar_sdk.git")
set(_RPLIDAR_GIT_REF "master")

fetch_thirdparty(NAME rplidar GIT_REPO "${_RPLIDAR_GIT_REPO}" GIT_REF "${_RPLIDAR_GIT_REF}" OUT_SOURCE_DIR _RPLIDAR_ROOT)

# Repo has sdk/ (and sometimes sdk/sdk/)
set(_RPLIDAR_SDK_SRC "${_RPLIDAR_ROOT}/sdk")
if(NOT EXISTS "${_RPLIDAR_SDK_SRC}/include/sl_lidar.h" AND EXISTS "${_RPLIDAR_SDK_SRC}/sdk/include/sl_lidar.h")
  set(_RPLIDAR_SDK_SRC "${_RPLIDAR_SDK_SRC}/sdk")
endif()

if(NOT EXISTS "${_RPLIDAR_SDK_SRC}/include/sl_lidar.h")
  message(FATAL_ERROR "drv_uart_rplidar: rplidar SDK does not provide sl_* API (missing include/sl_lidar.h) under ${_RPLIDAR_ROOT}")
endif()

set(_RPLIDAR_SOURCES
  ${_RPLIDAR_SDK_SRC}/src/sl_lidar_driver.cpp
  ${_RPLIDAR_SDK_SRC}/src/sl_crc.cpp
  ${_RPLIDAR_SDK_SRC}/src/sl_serial_channel.cpp
  ${_RPLIDAR_SDK_SRC}/src/sl_tcp_channel.cpp
  ${_RPLIDAR_SDK_SRC}/src/sl_udp_channel.cpp
  ${_RPLIDAR_SDK_SRC}/src/sl_lidarprotocol_codec.cpp
  ${_RPLIDAR_SDK_SRC}/src/sl_async_transceiver.cpp
  ${_RPLIDAR_SDK_SRC}/src/hal/thread.cpp
  ${_RPLIDAR_SDK_SRC}/src/dataunpacker/dataunpacker.cpp
  ${_RPLIDAR_SDK_SRC}/src/arch/linux/net_serial.cpp
  ${_RPLIDAR_SDK_SRC}/src/arch/linux/net_socket.cpp
  ${_RPLIDAR_SDK_SRC}/src/arch/linux/timer.cpp
)
file(GLOB _RPLIDAR_UNPACKER_SOURCES "${_RPLIDAR_SDK_SRC}/src/dataunpacker/unpacker/*.cpp")
list(APPEND _RPLIDAR_SOURCES ${_RPLIDAR_UNPACKER_SOURCES})

add_library(rplidar_sdk STATIC ${_RPLIDAR_SOURCES})
target_include_directories(rplidar_sdk
  PUBLIC
    $<BUILD_INTERFACE:${_RPLIDAR_SDK_SRC}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    "${_RPLIDAR_SDK_SRC}/src"
    "${_RPLIDAR_SDK_SRC}/src/hal"
    "${_RPLIDAR_SDK_SRC}/src/arch/linux"
    "${_RPLIDAR_SDK_SRC}/src/dataunpacker"
    "${_RPLIDAR_SDK_SRC}/src/dataunpacker/unpacker"
)
target_compile_features(rplidar_sdk PUBLIC cxx_std_11)
find_package(Threads REQUIRED)
target_link_libraries(rplidar_sdk PUBLIC Threads::Threads m)
target_compile_options(rplidar_sdk PRIVATE -w)
set_property(TARGET rplidar_sdk PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(components::rplidar_sdk ALIAS rplidar_sdk)
