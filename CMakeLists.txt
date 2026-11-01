# SPDX-License-Identifier: GPL-2.0-only
cmake_minimum_required(VERSION 3.16)
project(otr_userland CXX)

include(FetchContent)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Pinned dependencies (cmake/*.revision), the same arrangement as siano-userland.
# OTR_*_SOURCE_DIR overrides the fetch with a local tree.
set(OTR_DTV_CORE_SOURCE_DIR "" CACHE PATH "Local dtv-userland-core tree (override)")
set(OTR_PX4_USERLAND_SOURCE_DIR "" CACHE PATH "Local px4-userland tree (override)")

function(otr_read_revision name out)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${name}.revision" _rev)
    string(STRIP "${_rev}" _rev)
    string(LENGTH "${_rev}" _len)
    if(NOT _len EQUAL 40 OR NOT _rev MATCHES "^[0-9a-f]+$")
        message(FATAL_ERROR "cmake/${name}.revision must hold a full commit hash")
    endif()
    set(${out} "${_rev}" PARENT_SCOPE)
endfunction()
otr_read_revision(dtv-userland-core OTR_DTV_CORE_REVISION)
otr_read_revision(px4-userland OTR_PX4_REVISION)

if(OTR_DTV_CORE_SOURCE_DIR)
    set(OTR_DTV_CORE_SOURCE "${OTR_DTV_CORE_SOURCE_DIR}")
else()
    FetchContent_Declare(dtv_userland_core
        GIT_REPOSITORY "https://github.com/Khronos31/dtv-userland-core.git"
        GIT_TAG "${OTR_DTV_CORE_REVISION}")
    FetchContent_GetProperties(dtv_userland_core)
    if(NOT dtv_userland_core_POPULATED)
        FetchContent_Populate(dtv_userland_core)
    endif()
    set(OTR_DTV_CORE_SOURCE "${dtv_userland_core_SOURCE_DIR}")
endif()

if(OTR_PX4_USERLAND_SOURCE_DIR)
    set(OTR_PX4_SOURCE "${OTR_PX4_USERLAND_SOURCE_DIR}")
else()
    FetchContent_Declare(px4_userland
        GIT_REPOSITORY "https://github.com/Khronos31/px4-userland.git"
        GIT_TAG "${OTR_PX4_REVISION}")
    FetchContent_GetProperties(px4_userland)
    if(NOT px4_userland_POPULATED)
        FetchContent_Populate(px4_userland)
    endif()
    set(OTR_PX4_SOURCE "${px4_userland_SOURCE_DIR}")
endif()

add_subdirectory(${OTR_DTV_CORE_SOURCE} dtv_core EXCLUDE_FROM_ALL)

function(otr_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /EHs-c- /GR-)
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Werror -fno-exceptions -fno-rtti)
    endif()
endfunction()

# Chip drivers reused verbatim from the pinned px4-userland tree (GPL-2.0-only).
add_library(otr_px4_chips STATIC
    ${OTR_PX4_SOURCE}/userland/src/cxd2856er.cpp
    ${OTR_PX4_SOURCE}/userland/src/cxd2858er.cpp
    ${OTR_PX4_SOURCE}/userland/src/tagged_ts_demux.cpp)
target_include_directories(otr_px4_chips PUBLIC
    ${OTR_PX4_SOURCE}/userland/include ${OTR_PX4_SOURCE}/userland/src)
target_link_libraries(otr_px4_chips PUBLIC dtv::core)
target_compile_options(otr_px4_chips PRIVATE -fno-exceptions -fno-rtti)

# OTR-4TS3-U1 backend: FX3 vendor-request protocol (docs/design.md section 4).
find_package(Threads REQUIRED)
add_library(otr_backend STATIC src/fx3_i2c.cpp src/fx3_device.cpp)
target_include_directories(otr_backend PUBLIC src)
target_link_libraries(otr_backend PUBLIC otr_px4_chips Threads::Threads)
otr_warnings(otr_backend)

# libusb transport (the only part that needs libusb).
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0>=1.0.23)
add_library(otr_libusb STATIC src/libusb_backend.cpp)
target_link_libraries(otr_libusb PUBLIC otr_backend PkgConfig::LIBUSB)
otr_warnings(otr_libusb)

include(CTest)
if(BUILD_TESTING)
    add_executable(otr_backend_tests tests/fx3_i2c_tests.cpp)
    target_link_libraries(otr_backend_tests PRIVATE otr_backend)
    otr_warnings(otr_backend_tests)
    add_test(NAME otr_backend_tests COMMAND otr_backend_tests)

    add_executable(otr_device_tests tests/fx3_device_tests.cpp)
    target_link_libraries(otr_device_tests PRIVATE otr_backend)
    otr_warnings(otr_device_tests)
    add_test(NAME otr_device_tests COMMAND otr_device_tests)

    add_executable(otr_ts_path_tests tests/ts_path_tests.cpp)
    target_link_libraries(otr_ts_path_tests PRIVATE otr_px4_chips)
    otr_warnings(otr_ts_path_tests)
    add_test(NAME otr_ts_path_tests COMMAND otr_ts_path_tests)

    set_tests_properties(otr_backend_tests otr_device_tests otr_ts_path_tests
        PROPERTIES TIMEOUT 60)
endif()
