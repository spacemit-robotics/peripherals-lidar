# drv_uart_ydlidar: fetch ydlidar SDK and expose components::ydlidar_sdk (B2: single definition)
if(TARGET components::ydlidar_sdk)
  return()
endif()

get_filename_component(_LIDAR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
include("${_LIDAR_ROOT}/cmake/FetchThirdParty.cmake")

# Track upstream master (latest) managed by this driver
set(_YDLIDAR_GIT_REPO "https://github.com/YDLIDAR/YDLidar-SDK.git")
set(_YDLIDAR_GIT_REF "master")

fetch_thirdparty(NAME ydlidar GIT_REPO "${_YDLIDAR_GIT_REPO}" GIT_REF "${_YDLIDAR_GIT_REF}" OUT_SOURCE_DIR _YDLIDAR_SRC)

if(NOT TARGET ydlidar_sdk)
  set(BUILD_EXAMPLES OFF CACHE BOOL "ydlidar: build examples" FORCE)
  set(BUILD_TEST OFF CACHE BOOL "ydlidar: build tests" FORCE)
  set(BUILD_CSHARP OFF CACHE BOOL "ydlidar: build csharp" FORCE)
  add_subdirectory("${_YDLIDAR_SRC}" "${CMAKE_BINARY_DIR}/thirdparty/ydlidar" EXCLUDE_FROM_ALL)
endif()

if(NOT TARGET ydlidar_sdk)
  message(FATAL_ERROR "drv_uart_ydlidar: expected ydlidar_sdk target after add_subdirectory")
endif()

# Upstream only exposes src/; ydlidar_sdk.h includes <core/common/ydlidar_def.h>, so add repo root.
target_include_directories(ydlidar_sdk PUBLIC "${_YDLIDAR_SRC}")

# Upstream uses plain target_link_libraries; do not add keyword form here.
set_property(TARGET ydlidar_sdk PROPERTY POSITION_INDEPENDENT_CODE ON)

if(NOT TARGET components::ydlidar_sdk)
  add_library(components::ydlidar_sdk ALIAS ydlidar_sdk)
endif()
