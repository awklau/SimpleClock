add_executable(simpleclock_crc16_tests
    test_crc16.c
)

add_executable(simpleclock_gnss_control_tests test_gnss_control.c)
target_link_libraries(simpleclock_gnss_control_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_gnss_control_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_gnss_control_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_gnss_control COMMAND simpleclock_gnss_control_tests)

target_link_libraries(simpleclock_crc16_tests PRIVATE simpleclock_core)
target_compile_features(simpleclock_crc16_tests PRIVATE c_std_17)
set_target_properties(simpleclock_crc16_tests PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

if(MSVC)
    target_compile_options(simpleclock_crc16_tests PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(simpleclock_crc16_tests PRIVATE
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

add_test(NAME simpleclock_crc16 COMMAND simpleclock_crc16_tests)

add_executable(simpleclock_tmp117_tests
    test_tmp117.c
)

target_link_libraries(simpleclock_tmp117_tests PRIVATE simpleclock_core)
target_compile_features(simpleclock_tmp117_tests PRIVATE c_std_17)
set_target_properties(simpleclock_tmp117_tests PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

if(MSVC)
    target_compile_options(simpleclock_tmp117_tests PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(simpleclock_tmp117_tests PRIVATE
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

add_test(NAME simpleclock_tmp117 COMMAND simpleclock_tmp117_tests)

add_executable(simpleclock_sit5801_tests test_sit5801.c)
target_link_libraries(simpleclock_sit5801_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_sit5801_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_sit5801_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_sit5801 COMMAND simpleclock_sit5801_tests)
add_executable(simpleclock_text_tests test_text.c)
target_link_libraries(simpleclock_text_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_text_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_text_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_text COMMAND simpleclock_text_tests)
add_executable(simpleclock_sbf_tests test_sbf.c)
target_link_libraries(simpleclock_sbf_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_sbf_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_sbf_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_sbf COMMAND simpleclock_sbf_tests)
add_executable(simpleclock_sbf_fields_tests test_sbf_fields.c)
target_link_libraries(simpleclock_sbf_fields_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_sbf_fields_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_sbf_fields_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_sbf_fields COMMAND simpleclock_sbf_fields_tests)
add_executable(simpleclock_timing_tests test_timing.c)
target_link_libraries(simpleclock_timing_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_timing_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_timing_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_timing COMMAND simpleclock_timing_tests)
add_executable(simpleclock_bridge_tests test_bridge.c)
target_link_libraries(simpleclock_bridge_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_bridge_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_bridge_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_bridge COMMAND simpleclock_bridge_tests)

foreach(component IN ITEMS temperature heater preheat thermal_control thermal_response thermal_pid_response thermal_boot status_leds gnss_command gnss_control_lists fcw_trial hybrid hybrid_modes hybrid_handover hybrid_stats hybrid_input hybrid_faults hybrid_standalone hybrid_robust frequency_monitor thermal_outer settings)
    add_executable(simpleclock_${component}_tests test_${component}.c)
    target_link_libraries(simpleclock_${component}_tests PRIVATE simpleclock_core)
    if(MSVC)
        target_compile_options(simpleclock_${component}_tests PRIVATE /W4 /WX)
    else()
        target_compile_options(simpleclock_${component}_tests PRIVATE
            -Wall -Wextra -Wconversion -Wsign-conversion -Werror)
    endif()
    add_test(NAME simpleclock_${component} COMMAND simpleclock_${component}_tests)
endforeach()

# Compile the real M4 adapter against a test-only HAL boundary. These stubs are
# never included by firmware targets and make no electrical/timing assertions.
add_executable(simpleclock_thermal_probe_tests test_thermal_probe.c
    ../platform/stm32c071/src/thermal_probe.c)
target_include_directories(simpleclock_thermal_probe_tests PRIVATE
    stubs ../platform/stm32c071/include)
target_compile_definitions(simpleclock_thermal_probe_tests PRIVATE
    SIMPLECLOCK_CLOCK_PROBE=1 SIMPLECLOCK_THERMAL_PROBE=1)
target_link_libraries(simpleclock_thermal_probe_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_thermal_probe_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_thermal_probe_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_thermal_probe COMMAND simpleclock_thermal_probe_tests)
foreach(scenario IN ITEMS normal wrap range missing manual off stop fault ceiling)
    add_test(NAME simpleclock_thermal_boot_${scenario}
        COMMAND simpleclock_thermal_probe_tests ${scenario})
endforeach()

add_executable(simpleclock_gnss_probe_tests test_gnss_probe.c
    ../platform/stm32c071/src/gnss_probe.c)
target_include_directories(simpleclock_gnss_probe_tests PRIVATE
    stubs ../platform/stm32c071/include)
target_compile_definitions(simpleclock_gnss_probe_tests PRIVATE SIMPLECLOCK_CLOCK_PROBE=1)
target_link_libraries(simpleclock_gnss_probe_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_gnss_probe_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_gnss_probe_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_gnss_probe COMMAND simpleclock_gnss_probe_tests)

add_executable(simpleclock_gnss_auto_tests test_gnss_auto.c stubs/settings_flash.c
    ../platform/stm32c071/src/gnss_probe.c)
target_include_directories(simpleclock_gnss_auto_tests PRIVATE
    stubs ../platform/stm32c071/include)
target_compile_definitions(simpleclock_gnss_auto_tests PRIVATE
    SIMPLECLOCK_CLOCK_PROBE=1 SIMPLECLOCK_AUTO_HYBRID=1)
target_link_libraries(simpleclock_gnss_auto_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_gnss_auto_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_gnss_auto_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_gnss_auto COMMAND simpleclock_gnss_auto_tests)
add_test(NAME simpleclock_gnss_auto_headless COMMAND simpleclock_gnss_auto_tests headless)
add_test(NAME simpleclock_gnss_auto_reconnect COMMAND simpleclock_gnss_auto_tests reconnect)
add_test(NAME simpleclock_gnss_auto_early_reply COMMAND simpleclock_gnss_auto_tests early-reply)
add_test(NAME simpleclock_gnss_auto_cr_reply COMMAND simpleclock_gnss_auto_tests cr-reply)
foreach(scenario save wrap pending terminal overflow stall error)
    add_test(NAME simpleclock_gnss_dma_${scenario} COMMAND simpleclock_gnss_auto_tests dma-${scenario})
endforeach()
foreach(scenario direct system missing baud duplicate pps-duplicate)
    add_test(NAME simpleclock_gnss_defaults_${scenario} COMMAND simpleclock_gnss_auto_tests defaults-${scenario})
endforeach()
foreach(scenario static-auto static-fixed static-has static-sbas static-edit static-sbas-edit usb-before usb-completion usb-read-busy usb-partial-write)
    add_test(NAME simpleclock_gnss_${scenario} COMMAND simpleclock_gnss_auto_tests ${scenario})
endforeach()
foreach(scenario headless clear other-error stale invalid-epoch no-ref fresh reboot config-fault busy persistent timeout reject)
    add_test(NAME simpleclock_gnss_latch_${scenario} COMMAND simpleclock_gnss_auto_tests latch-${scenario})
endforeach()
foreach(scenario timeout recheck config-fault busy)
    add_test(NAME simpleclock_gnss_managed_${scenario} COMMAND simpleclock_gnss_auto_tests managed-${scenario})
endforeach()
foreach(scenario reply-burst reply-burst-busy reply-burst-mixed reply-burst-headless)
    add_test(NAME simpleclock_gnss_${scenario} COMMAND simpleclock_gnss_auto_tests ${scenario})
endforeach()
foreach(scenario cold qualified saved-off manual retry duplicate ignored shape report)
    add_test(NAME simpleclock_gnss_pps_policy_${scenario}
        COMMAND simpleclock_gnss_auto_tests pps-policy-${scenario})
endforeach()
add_test(NAME simpleclock_gnss_reboot COMMAND simpleclock_gnss_auto_tests reboot)
add_test(NAME simpleclock_gnss_reboot_rejected COMMAND simpleclock_gnss_auto_tests reboot-reject)
foreach(scenario IN ITEMS control control-ignore control-corrupt control-save-fail control-mcu-fail control-noop-cache lists lists-corrupt lists-dependency)
    add_test(NAME simpleclock_gnss_bridge_${scenario} COMMAND simpleclock_gnss_auto_tests ${scenario})
endforeach()
foreach(selection auto sbas has standalone)
    add_test(NAME simpleclock_gnss_saved_${selection} COMMAND simpleclock_gnss_auto_tests saved-${selection})
endforeach()
foreach(field clock timing data stream)
    add_test(NAME simpleclock_gnss_auto_missing_${field}
        COMMAND simpleclock_gnss_auto_tests missing ${field})
    add_test(NAME simpleclock_gnss_auto_duplicate_${field}
        COMMAND simpleclock_gnss_auto_tests duplicate ${field})
    add_test(NAME simpleclock_gnss_auto_pre_echo_${field}
        COMMAND simpleclock_gnss_auto_tests pre-echo ${field})
endforeach()

add_executable(simpleclock_fixed_thermal_tests test_thermal_probe.c
    ../platform/stm32c071/src/thermal_probe.c)
target_include_directories(simpleclock_fixed_thermal_tests PRIVATE
    stubs ../platform/stm32c071/include)
target_compile_definitions(simpleclock_fixed_thermal_tests PRIVATE
    SIMPLECLOCK_CLOCK_PROBE=1 SIMPLECLOCK_THERMAL_PROBE=1 SIMPLECLOCK_ACTUATOR_TEST=1)
target_link_libraries(simpleclock_fixed_thermal_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_fixed_thermal_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_fixed_thermal_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME simpleclock_fixed_thermal COMMAND simpleclock_fixed_thermal_tests fixed)

add_executable(simpleclock_auto_thermal_tests test_thermal_probe.c
    ../platform/stm32c071/src/thermal_probe.c)
target_include_directories(simpleclock_auto_thermal_tests PRIVATE
    stubs ../platform/stm32c071/include)
target_compile_definitions(simpleclock_auto_thermal_tests PRIVATE
    SIMPLECLOCK_CLOCK_PROBE=1 SIMPLECLOCK_THERMAL_PROBE=1 SIMPLECLOCK_AUTO_HYBRID=1)
target_link_libraries(simpleclock_auto_thermal_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_auto_thermal_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_auto_thermal_tests PRIVATE -Wall -Wextra -Werror)
endif()
foreach(scenario IN ITEMS baseline low_clamp high_clamp nohost trip70 decay cancel continuous storage_inhibit)
    add_test(NAME simpleclock_auto_thermal_${scenario}
        COMMAND simpleclock_auto_thermal_tests ${scenario})
endforeach()


add_executable(simpleclock_actuator_probe_tests test_actuator_probe.c
    ../platform/stm32c071/src/actuator_probe.c)
target_include_directories(simpleclock_actuator_probe_tests PRIVATE
    stubs ../platform/stm32c071/include)
target_compile_definitions(simpleclock_actuator_probe_tests PRIVATE
    SIMPLECLOCK_CLOCK_PROBE=1 SIMPLECLOCK_THERMAL_PROBE=1 SIMPLECLOCK_ACTUATOR_TEST=1)
target_link_libraries(simpleclock_actuator_probe_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_actuator_probe_tests PRIVATE /W4 /WX)
else()
    target_compile_options(simpleclock_actuator_probe_tests PRIVATE -Wall -Wextra -Werror)
endif()
foreach(scenario IN ITEMS normal before missing stale_cov unqualified abort disconnect partial readback usb)
    add_test(NAME simpleclock_actuator_${scenario}
        COMMAND simpleclock_actuator_probe_tests ${scenario})
endforeach()

add_executable(simpleclock_hybrid_probe_tests test_hybrid_probe.c stubs/settings_flash.c
    ../platform/stm32c071/src/hybrid_probe.c)
target_include_directories(simpleclock_hybrid_probe_tests PRIVATE
    stubs ../platform/stm32c071/include)
target_compile_definitions(simpleclock_hybrid_probe_tests PRIVATE
    SIMPLECLOCK_AUTO_HYBRID=1 SIMPLECLOCK_CLOCK_PROBE=1 SIMPLECLOCK_THERMAL_PROBE=1)
target_link_libraries(simpleclock_hybrid_probe_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_hybrid_probe_tests PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(simpleclock_hybrid_probe_tests PRIVATE
        -Wall -Wextra -Wconversion -Wsign-conversion -Wshadow -Werror)
endif()
foreach(scenario IN ITEMS normal standalone static_pvt usb_commands reboot_usb_timeout integrity_loss null_latest_freeze usb_parts partial busy bad_readback sticky_hold sensor_fault duty_gap startup_recovery frame_snapshot_race runtime_loops steering_commands reboot reboot_pending reboot_pending_bad reboot_pending_timeout defaults resume_recheck)
    add_test(NAME simpleclock_hybrid_probe_${scenario}
        COMMAND simpleclock_hybrid_probe_tests ${scenario})
endforeach()

# Combined real-controller / synthetic thermal plant regression.  Its optional
# CSV replay checks historical telemetry gates only; it is not closed-loop
# hardware performance evidence.
add_executable(simpleclock_hybrid_plant_tests test_hybrid_plant.c)
target_link_libraries(simpleclock_hybrid_plant_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_hybrid_plant_tests PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(simpleclock_hybrid_plant_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wundef
        -Wdouble-promotion -Wformat=2 -Werror)
endif()
add_test(NAME simpleclock_hybrid_plant COMMAND simpleclock_hybrid_plant_tests)

# Real clock/thermal controllers coupled through an explicitly synthetic,
# positive-gain thermal plant with two gain and lag mismatch cases.
add_executable(simpleclock_hybrid_coupled_tests test_hybrid_coupled.c)
target_link_libraries(simpleclock_hybrid_coupled_tests PRIVATE simpleclock_core)
if(MSVC)
    target_compile_options(simpleclock_hybrid_coupled_tests PRIVATE /W4 /WX /permissive-)
else()
    target_compile_options(simpleclock_hybrid_coupled_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wundef
        -Wdouble-promotion -Wformat=2 -Werror)
endif()
add_test(NAME simpleclock_hybrid_coupled COMMAND simpleclock_hybrid_coupled_tests)
