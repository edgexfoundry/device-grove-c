cmake_minimum_required(VERSION 3.0)
project(device_grove LANGUAGES C)

# Package support
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Versioning
file (STRINGS ${CMAKE_SOURCE_DIR}/../../VERSION GROVE_DOT_VERSION)

find_package (LibMRAA REQUIRED)
if (NOT LIBMRAA_FOUND)
  message (WARNING "mraa library or header not found")
endif ()

find_package (Libcsdk REQUIRED)
if (NOT LIBCSDK_FOUND)
  message (WARNING "csdk library or header not found")
endif ()

# Set additional C compiler flags
if (DEFINED CMAKE_C_COMPILER_ID)
  if (CMAKE_C_COMPILER_ID MATCHES GNU)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wmissing-prototypes")
    set (CMAKE_C_STANDARD 99)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_GNU_SOURCE")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
    set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -ggdb -O0")
    set (CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -ggdb -O0")
    set (CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG")
    set (CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -O3")
   endif ()
endif ()

file (GLOB C_FILES *.c)
find_library(EDGEX_CSDK_RELEASE_LIB NAMES csdk)
find_library(LIBMRAA_LIB NAMES mraa)
add_executable(device-grove ${C_FILES})
target_compile_definitions(device-grove PRIVATE GROVE_VERSION="${GROVE_DOT_VERSION}")
target_include_directories (device-grove PRIVATE . ${LIBMRAA_INCLUDE_DIR} ${LIBCSDK_INCLUDE_DIR})
target_link_libraries (device-grove PUBLIC m PRIVATE ${LIBMRAA_LIB} ${EDGEX_CSDK_RELEASE_LIB})
install(TARGETS device-grove DESTINATION bin)
