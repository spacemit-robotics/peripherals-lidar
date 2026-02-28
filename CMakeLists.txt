cmake_minimum_required(VERSION 3.10)
project(lidar LANGUAGES C CXX)

# Standalone default (and compatibility with exported targets).
if(NOT DEFINED SROBOTIS_EXPORT_SET)
    set(SROBOTIS_EXPORT_SET "ComponentsTargets" CACHE STRING "Export set name used by exported targets")
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enabled drivers: SDK sets SROBOTIS_PERIPHERALS_LIDAR_ENABLED_DRIVERS; standalone uses LIDAR_ENABLED_DRIVERS.
if(DEFINED SROBOTIS_PERIPHERALS_LIDAR_ENABLED_DRIVERS)
    set(_lidar_enabled_drivers "${SROBOTIS_PERIPHERALS_LIDAR_ENABLED_DRIVERS}")
else()
    set(LIDAR_ENABLED_DRIVERS "drv_uart_ydlidar;drv_uart_rplidar" CACHE STRING "Enabled lidar drivers (semicolon-separated)")
    set(_lidar_enabled_drivers "${LIDAR_ENABLED_DRIVERS}")
endif()

# Fetch/build thirdparty per driver (scheme B: each driver's driver.cmake does fetch + build).
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchThirdParty.cmake")
list(FIND _lidar_enabled_drivers "drv_uart_ydlidar" _ydlidar_idx)
if(NOT _ydlidar_idx EQUAL -1)
    include("${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/drv_uart_ydlidar/driver.cmake")
endif()
list(FIND _lidar_enabled_drivers "drv_uart_rplidar" _rplidar_idx)
if(NOT _rplidar_idx EQUAL -1)
    include("${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/drv_uart_rplidar/driver.cmake")
endif()

# Collect sources based on enabled driver list.
set(_lidar_sources
    src/lidar_core.c
)

function(_lidar_append_driver_source out_var drv)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/${drv}.c")
        list(APPEND "${out_var}" "src/drivers/${drv}.c")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/${drv}.cpp")
        list(APPEND "${out_var}" "src/drivers/${drv}.cpp")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/${drv}/${drv}.c")
        list(APPEND "${out_var}" "src/drivers/${drv}/${drv}.c")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/${drv}/${drv}.cpp")
        list(APPEND "${out_var}" "src/drivers/${drv}/${drv}.cpp")
    else()
        message(FATAL_ERROR "lidar: enabled driver '${drv}' not found at src/drivers/${drv}.c|.cpp or src/drivers/${drv}/${drv}.c|.cpp")
    endif()
    set("${out_var}" "${${out_var}}" PARENT_SCOPE)
endfunction()

foreach(_drv IN LISTS _lidar_enabled_drivers)
    _lidar_append_driver_source(_lidar_sources "${_drv}")
endforeach()

add_library(lidar SHARED ${_lidar_sources})

target_include_directories(lidar
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

if(NOT _ydlidar_idx EQUAL -1)
    target_link_libraries(lidar PRIVATE components::ydlidar_sdk)
    set_property(TARGET lidar PROPERTY LINKER_LANGUAGE CXX)
endif()
if(NOT _rplidar_idx EQUAL -1)
    target_link_libraries(lidar PRIVATE components::rplidar_sdk)
    set_property(TARGET lidar PROPERTY LINKER_LANGUAGE CXX)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lidar PRIVATE -Wall -Wextra)
endif()

install(TARGETS lidar
    EXPORT ${SROBOTIS_EXPORT_SET}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(FILES include/lidar.h DESTINATION include)

# Optional tests
option(LIDAR_BUILD_TESTS "Build lidar test programs" ON)
if(LIDAR_BUILD_TESTS)
    add_executable(test_lidar_uart test/test_lidar_uart.c)
    target_link_libraries(test_lidar_uart lidar)
    set_target_properties(test_lidar_uart PROPERTIES BUILD_RPATH "$ORIGIN/../lib")
    install(TARGETS test_lidar_uart RUNTIME DESTINATION bin)
endif()
