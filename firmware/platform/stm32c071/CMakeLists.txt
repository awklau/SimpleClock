set(STM32C071_DIAGNOSTIC_TARGET simpleclock_stm32c071_led1)
set(STM32C071_VENDOR_ROOT "${PROJECT_SOURCE_DIR}/vendor/st")
set(STM32C071_DEVICE_ROOT "${STM32C071_VENDOR_ROOT}/cmsis-device-c0")
set(STM32C071_CMSIS_ROOT "${STM32C071_VENDOR_ROOT}/cmsis")
set(STM32C071_ARTIFACT_DIR "${CMAKE_BINARY_DIR}/artifacts")
set(STM32C071_ELF "${STM32C071_ARTIFACT_DIR}/stm32c071_led1.elf")
set(STM32C071_BIN "${STM32C071_ARTIFACT_DIR}/stm32c071_led1.bin")
set(STM32C071_HEX "${STM32C071_ARTIFACT_DIR}/stm32c071_led1.hex")
set(STM32C071_MAP "${STM32C071_ARTIFACT_DIR}/stm32c071_led1.map")
set(STM32C071_SIZE "${STM32C071_ARTIFACT_DIR}/stm32c071_led1.size.txt")
set(STM32C071_LINKER_SCRIPT
    "${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32c071_flash.ld")

enable_language(ASM)
add_compile_options(-Os -fstack-usage)

add_executable(${STM32C071_DIAGNOSTIC_TARGET}
    src/main.c
    "${STM32C071_DEVICE_ROOT}/Source/Templates/system_stm32c0xx.c"
    "${STM32C071_DEVICE_ROOT}/Source/Templates/gcc/startup_stm32c071xx.s"
)

set_target_properties(${STM32C071_DIAGNOSTIC_TARGET} PROPERTIES
    OUTPUT_NAME stm32c071_led1
    RUNTIME_OUTPUT_DIRECTORY "${STM32C071_ARTIFACT_DIR}"
    SUFFIX .elf
    LINK_DEPENDS "${STM32C071_LINKER_SCRIPT};${CMAKE_CURRENT_SOURCE_DIR}/size-report.cmake"
)

target_compile_definitions(${STM32C071_DIAGNOSTIC_TARGET} PRIVATE
    STM32C071xx
)

target_include_directories(${STM32C071_DIAGNOSTIC_TARGET} PRIVATE
    "${STM32C071_DEVICE_ROOT}/Include"
    "${STM32C071_CMSIS_ROOT}/Include"
)

target_compile_options(${STM32C071_DIAGNOSTIC_TARGET} PRIVATE
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -fno-common
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

target_link_options(${STM32C071_DIAGNOSTIC_TARGET} PRIVATE
    -mcpu=cortex-m0plus
    -mthumb
    -mfloat-abi=soft
    -nostartfiles
    --specs=nano.specs
    --specs=nosys.specs
    "-T${STM32C071_LINKER_SCRIPT}"
    "-Wl,--Map=${STM32C071_MAP}"
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

add_custom_command(TARGET ${STM32C071_DIAGNOSTIC_TARGET} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${STM32C071_ARTIFACT_DIR}"
    COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${STM32C071_DIAGNOSTIC_TARGET}>" "${STM32C071_BIN}"
    COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${STM32C071_DIAGNOSTIC_TARGET}>" "${STM32C071_HEX}"
    COMMAND ${CMAKE_COMMAND}
        -DARM_SIZE:FILEPATH=${CMAKE_SIZE}
        -DELF_FILE:FILEPATH=$<TARGET_FILE:${STM32C071_DIAGNOSTIC_TARGET}>
        -DSIZE_FILE:FILEPATH=${STM32C071_SIZE}
        -P "${CMAKE_CURRENT_SOURCE_DIR}/size-report.cmake"
    VERBATIM
)

add_custom_target(${STM32C071_DIAGNOSTIC_TARGET}_size
    DEPENDS ${STM32C071_DIAGNOSTIC_TARGET}
    COMMAND ${CMAKE_COMMAND} -E cat "${STM32C071_SIZE}"
    VERBATIM
)

if(SIMPLECLOCK_BUILD_STM32C071_TMP117_CDC)
    if(NOT SIMPLECLOCK_PRIVATE_USB_TEST_IDENTITY_ACK)
        message(FATAL_ERROR
            "TMP117 CDC target requires SIMPLECLOCK_PRIVATE_USB_TEST_IDENTITY_ACK=1; "
            "VID:PID 1209:0001 is private bench identity only")
    endif()

    set(STM32C071_TMP117_TARGET simpleclock_stm32c071_tmp117_cdc)
    set(STM32C071_TMP117_ARTIFACT_DIR "${CMAKE_BINARY_DIR}/artifacts")
    set(STM32C071_TMP117_ELF
        "${STM32C071_TMP117_ARTIFACT_DIR}/stm32c071_tmp117_cdc.elf")
    set(STM32C071_TMP117_BIN
        "${STM32C071_TMP117_ARTIFACT_DIR}/stm32c071_tmp117_cdc.bin")
    set(STM32C071_TMP117_HEX
        "${STM32C071_TMP117_ARTIFACT_DIR}/stm32c071_tmp117_cdc.hex")
    set(STM32C071_TMP117_MAP
        "${STM32C071_TMP117_ARTIFACT_DIR}/stm32c071_tmp117_cdc.map")
    set(STM32C071_TMP117_SIZE
        "${STM32C071_TMP117_ARTIFACT_DIR}/stm32c071_tmp117_cdc.size.txt")
    set(STM32C071_TMP117_LINKER_SCRIPT
        "${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32c071_flash.ld")

    set(STM32C071_HAL_HEADERS
        "${STM32C071_VENDOR_ROOT}/hal/Inc"
    )
    set(STM32C071_HAL_SOURCES
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_cortex.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_gpio.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_i2c.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_pcd.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_pcd_ex.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_pwr.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_pwr_ex.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_rcc.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_hal_rcc_ex.c"
        "${STM32C071_VENDOR_ROOT}/hal/Src/stm32c0xx_ll_usb.c"
    )

    add_library(simpleclock_stm32c071_hal STATIC ${STM32C071_HAL_SOURCES})
    target_compile_definitions(simpleclock_stm32c071_hal PRIVATE STM32C071xx)
    target_include_directories(simpleclock_stm32c071_hal
        PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${STM32C071_DEVICE_ROOT}/Include"
            "${STM32C071_CMSIS_ROOT}/Include"
            ${STM32C071_HAL_HEADERS}
    )
    target_compile_options(simpleclock_stm32c071_hal PRIVATE
        -mcpu=cortex-m0plus
        -mthumb
        -mfloat-abi=soft
        -ffunction-sections
        -fdata-sections
        -fno-common
        -w
    )

    set(STM32C071_USB_SOURCES
        "${STM32C071_VENDOR_ROOT}/usb-device/Core/Src/usbd_core.c"
        "${STM32C071_VENDOR_ROOT}/usb-device/Core/Src/usbd_ctlreq.c"
        "${STM32C071_VENDOR_ROOT}/usb-device/Core/Src/usbd_ioreq.c"
        "${STM32C071_VENDOR_ROOT}/usb-device/Class/CDC/Src/usbd_cdc.c"
    )
    add_library(simpleclock_stm32c071_usb STATIC ${STM32C071_USB_SOURCES})
    target_compile_definitions(simpleclock_stm32c071_usb PRIVATE
        STM32C071xx
        SIMPLECLOCK_PRIVATE_USB_TEST_IDENTITY_ACK=1
    )
    target_include_directories(simpleclock_stm32c071_usb
        PUBLIC
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${STM32C071_DEVICE_ROOT}/Include"
            "${STM32C071_CMSIS_ROOT}/Include"
            ${STM32C071_HAL_HEADERS}
            "${STM32C071_VENDOR_ROOT}/usb-device/Core/Inc"
            "${STM32C071_VENDOR_ROOT}/usb-device/Class/CDC/Inc"
    )
    target_compile_options(simpleclock_stm32c071_usb PRIVATE
        -mcpu=cortex-m0plus
        -mthumb
        -mfloat-abi=soft
        -ffunction-sections
        -fdata-sections
        -fno-common
        -w
    )

    # The binary bridge and ASCII diagnostic own the same UART/USB callbacks.
    # Select one application and CDC adapter; never link both into an image.
    # All CDC presets retain the historical stm32c071_tmp117_cdc artifact name,
    # so the preset-specific build directory is part of the image identity.
    if(SIMPLECLOCK_GNSS_BRIDGE)
        if(SIMPLECLOCK_CLOCK_PROBE)
            message(FATAL_ERROR "Bridge and diagnostic profiles are mutually exclusive")
        endif()
        set(STM32C071_APPLICATION_SOURCES src/bridge_main.c src/bridge_service.c)
        set(STM32C071_CDC_SOURCE src/bridge_usb.c)
    else()
        set(STM32C071_APPLICATION_SOURCES src/tmp117_cdc_main.c)
        set(STM32C071_CDC_SOURCE src/usbd_cdc_if.c)
    endif()
    add_executable(${STM32C071_TMP117_TARGET}
        ${STM32C071_APPLICATION_SOURCES}
        src/tmp117_cdc_it.c
        src/usb_device.c
        ${STM32C071_CDC_SOURCE}
        src/usbd_conf.c
        src/usbd_desc.c
        "${STM32C071_DEVICE_ROOT}/Source/Templates/system_stm32c0xx.c"
        "${STM32C071_DEVICE_ROOT}/Source/Templates/gcc/startup_stm32c071xx.s"
    )

    set_target_properties(${STM32C071_TMP117_TARGET} PROPERTIES
        OUTPUT_NAME stm32c071_tmp117_cdc
        RUNTIME_OUTPUT_DIRECTORY "${STM32C071_TMP117_ARTIFACT_DIR}"
        SUFFIX .elf
        LINK_DEPENDS "${STM32C071_TMP117_LINKER_SCRIPT};${CMAKE_CURRENT_SOURCE_DIR}/size-report.cmake"
    )
    target_compile_definitions(${STM32C071_TMP117_TARGET} PRIVATE
        STM32C071xx
        SIMPLECLOCK_PRIVATE_USB_TEST_IDENTITY_ACK=1
    )
    if(SIMPLECLOCK_GNSS_BRIDGE)
        target_compile_definitions(${STM32C071_TMP117_TARGET} PRIVATE SIMPLECLOCK_GNSS_BRIDGE=1)
    endif()
    if(SIMPLECLOCK_CLOCK_PROBE)
        target_sources(${STM32C071_TMP117_TARGET} PRIVATE src/clock_probe.c src/gnss_probe.c src/timing_probe.c)
        target_compile_definitions(${STM32C071_TMP117_TARGET} PRIVATE SIMPLECLOCK_CLOCK_PROBE=1)
    endif()
    if(SIMPLECLOCK_THERMAL_PROBE)
        target_sources(${STM32C071_TMP117_TARGET} PRIVATE src/thermal_probe.c)
        target_compile_definitions(${STM32C071_TMP117_TARGET} PRIVATE SIMPLECLOCK_THERMAL_PROBE=1)
    endif()
    if(SIMPLECLOCK_ACTUATOR_TEST)
        target_sources(${STM32C071_TMP117_TARGET} PRIVATE src/actuator_probe.c)
        target_compile_definitions(${STM32C071_TMP117_TARGET} PRIVATE SIMPLECLOCK_ACTUATOR_TEST=1)
        target_link_options(${STM32C071_TMP117_TARGET} PRIVATE "-Wl,--defsym=__actuator_diagnostic=1")
    endif()
    if(SIMPLECLOCK_AUTO_HYBRID)
        target_sources(${STM32C071_TMP117_TARGET} PRIVATE src/hybrid_probe.c src/settings_flash.c)
        target_compile_definitions(${STM32C071_TMP117_TARGET} PRIVATE SIMPLECLOCK_AUTO_HYBRID=1)
        target_link_options(${STM32C071_TMP117_TARGET} PRIVATE "-Wl,--defsym=__hybrid_release=1")
    endif()
    target_include_directories(${STM32C071_TMP117_TARGET} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${STM32C071_DEVICE_ROOT}/Include"
        "${STM32C071_CMSIS_ROOT}/Include"
        ${STM32C071_HAL_HEADERS}
        "${STM32C071_VENDOR_ROOT}/usb-device/Core/Inc"
        "${STM32C071_VENDOR_ROOT}/usb-device/Class/CDC/Inc"
        "${PROJECT_SOURCE_DIR}/core/include"
    )
    target_compile_options(${STM32C071_TMP117_TARGET} PRIVATE
        -mcpu=cortex-m0plus
        -mthumb
        -mfloat-abi=soft
        -ffunction-sections
        -fdata-sections
        -fno-common
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
    target_link_libraries(${STM32C071_TMP117_TARGET} PRIVATE
        simpleclock_core
        simpleclock_stm32c071_usb
        simpleclock_stm32c071_hal
    )
    target_link_options(${STM32C071_TMP117_TARGET} PRIVATE
        -mcpu=cortex-m0plus
        -mthumb
        -mfloat-abi=soft
        -nostartfiles
        --specs=nano.specs
        --specs=nosys.specs
        "-T${STM32C071_TMP117_LINKER_SCRIPT}"
        "-Wl,--Map=${STM32C071_TMP117_MAP}"
        -Wl,--gc-sections
        -Wl,--print-memory-usage
    )
    add_custom_command(TARGET ${STM32C071_TMP117_TARGET} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${STM32C071_TMP117_ARTIFACT_DIR}"
        COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${STM32C071_TMP117_TARGET}>"
            "${STM32C071_TMP117_BIN}"
        COMMAND ${CMAKE_OBJCOPY} -O ihex "$<TARGET_FILE:${STM32C071_TMP117_TARGET}>"
            "${STM32C071_TMP117_HEX}"
        COMMAND ${CMAKE_COMMAND}
            -DARM_SIZE:FILEPATH=${CMAKE_SIZE}
            -DELF_FILE:FILEPATH=$<TARGET_FILE:${STM32C071_TMP117_TARGET}>
            -DSIZE_FILE:FILEPATH=${STM32C071_TMP117_SIZE}
            -DAUTO_HYBRID:BOOL=${SIMPLECLOCK_AUTO_HYBRID}
            -DACTUATOR_TEST:BOOL=${SIMPLECLOCK_ACTUATOR_TEST}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/size-report.cmake"
        VERBATIM
    )
    add_custom_target(${STM32C071_TMP117_TARGET}_size
        DEPENDS ${STM32C071_TMP117_TARGET}
        COMMAND ${CMAKE_COMMAND} -E cat "${STM32C071_TMP117_SIZE}"
        VERBATIM
    )
endif()
