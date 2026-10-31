cmake_minimum_required(VERSION 3.25)

project(SimpleClock LANGUAGES C)

option(SIMPLECLOCK_BUILD_TESTS "Build host-side tests" ${PROJECT_IS_TOP_LEVEL})
option(SIMPLECLOCK_BUILD_STM32C071_LED1_DIAGNOSTIC
    "Build the STM32C071FBP6 PA8 LED1 diagnostic image" OFF)
option(SIMPLECLOCK_BUILD_STM32C071_TMP117_CDC
    "Build the private-bench STM32C071 TMP117 USB CDC diagnostic" OFF)
option(SIMPLECLOCK_PRIVATE_USB_TEST_IDENTITY_ACK
    "Acknowledge that USB 1209:0001 is private in-house test-only" OFF)
option(SIMPLECLOCK_CLOCK_PROBE "Add read-only assembled clock bring-up services" OFF)
option(SIMPLECLOCK_GNSS_BRIDGE "Build a quiet GNSS update maintenance bridge" OFF)
option(SIMPLECLOCK_THERMAL_PROBE "Add automatic PI startup and bounded thermal trials" OFF)
option(SIMPLECLOCK_ACTUATOR_TEST "Add the explicitly started bounded +1 ppb bench experiment" OFF)
option(SIMPLECLOCK_AUTO_HYBRID "Build the normal autonomous AUTO_HYBRID_V1 firmware profile" OFF)
if(SIMPLECLOCK_AUTO_HYBRID AND (NOT SIMPLECLOCK_THERMAL_PROBE OR SIMPLECLOCK_ACTUATOR_TEST))
    message(FATAL_ERROR "Autonomous release requires thermal services and exclusive FCW ownership")
endif()
if(SIMPLECLOCK_ACTUATOR_TEST AND NOT SIMPLECLOCK_THERMAL_PROBE)
    message(FATAL_ERROR "Actuator test requires the thermal diagnostic application")
endif()
if(SIMPLECLOCK_THERMAL_PROBE AND
   (NOT SIMPLECLOCK_CLOCK_PROBE OR NOT SIMPLECLOCK_BUILD_STM32C071_TMP117_CDC OR SIMPLECLOCK_GNSS_BRIDGE))
    message(FATAL_ERROR "Thermal probe requires the clock-probe CDC application, never the bridge")
endif()

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if(CMAKE_CROSSCOMPILING AND SIMPLECLOCK_BUILD_TESTS)
    message(FATAL_ERROR
        "SIMPLECLOCK_BUILD_TESTS=ON is not supported for cross compilation; "
        "configure the native preset to run host tests")
endif()

add_library(simpleclock_core STATIC
    core/src/crc16.c
    core/src/tmp117.c
    core/src/sit5801.c
    core/src/fcw_trial.c
    core/src/hybrid.c
    core/src/settings.c
    core/src/hybrid_stats.c
    core/src/hybrid_input.c
    core/src/frequency_monitor.c
    core/src/thermal_outer.c
    core/src/thermal_boot.c
    core/src/text.c
    core/src/sbf.c
    core/src/sbf_fields.c
    core/src/gnss_command.c
    core/src/gnss_control.c
    core/src/gnss_control_lists.c
    core/src/timing.c
    core/src/bridge.c
    core/src/temperature.c
    core/src/heater.c
    core/src/preheat.c
    core/src/thermal_control.c
    core/src/status_leds.c
)

target_compile_features(simpleclock_core PUBLIC c_std_17)
if(NOT MSVC)
    target_link_libraries(simpleclock_core PUBLIC m)
endif()
set_target_properties(simpleclock_core PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)
target_include_directories(simpleclock_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
)

# Keep warnings private to this project.  Consumers of the static library do
# not inherit project policy or compiler-specific options.
if(MSVC)
    target_compile_options(simpleclock_core PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(simpleclock_core PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wundef
        -Wdouble-promotion
        -Wformat=2
        -Werror
    )
endif()

if(CMAKE_CROSSCOMPILING)
    target_compile_options(simpleclock_core PRIVATE -Os -ffunction-sections -fdata-sections -fstack-usage)
endif()

if(SIMPLECLOCK_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(SIMPLECLOCK_BUILD_STM32C071_LED1_DIAGNOSTIC OR
   SIMPLECLOCK_BUILD_STM32C071_TMP117_CDC)
    if(NOT CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR
            "STM32C071 diagnostic targets require the Arm cross toolchain")
    endif()
    add_subdirectory(platform/stm32c071)
endif()
