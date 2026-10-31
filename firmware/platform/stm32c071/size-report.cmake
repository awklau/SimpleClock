if(NOT DEFINED ARM_SIZE OR NOT DEFINED ELF_FILE OR NOT DEFINED SIZE_FILE)
    message(FATAL_ERROR "ARM_SIZE, ELF_FILE, and SIZE_FILE are required")
endif()

get_filename_component(arm_bin "${ARM_SIZE}" DIRECTORY)
get_filename_component(elf_base "${ELF_FILE}" NAME_WE)
get_filename_component(elf_dir "${ELF_FILE}" DIRECTORY)
file(SIZE "${elf_dir}/${elf_base}.bin" flash_bytes)
set(flash_gate 49152)
set(static_gate 14336)
set(uncommitted_gate 6144)
if(ACTUATOR_TEST)
    set(flash_gate 53248)
endif()
if(AUTO_HYBRID)
    set(flash_gate 114688)
    set(static_gate 19456)
    set(uncommitted_gate 1024)
endif()
if(flash_bytes GREATER flash_gate)
    message(FATAL_ERROR "Loadable binary ${flash_bytes} bytes exceeds ${flash_gate} byte profile gate")
endif()
foreach(tool IN ITEMS size nm readelf)
    if(tool STREQUAL "size")
        set(args -A)
        set(suffix sections)
    elseif(tool STREQUAL "nm")
        set(args --print-size --size-sort --radix=d)
        set(suffix nm)
    else()
        set(args -lW)
        set(suffix readelf)
    endif()
    execute_process(COMMAND "${arm_bin}/arm-none-eabi-${tool}" ${args} "${ELF_FILE}"
        OUTPUT_VARIABLE report RESULT_VARIABLE report_result)
    if(NOT report_result EQUAL 0)
        message(FATAL_ERROR "Resource report ${tool} failed")
    endif()
    file(WRITE "${elf_dir}/${elf_base}.${suffix}.txt" "${report}")
endforeach()
file(WRITE "${elf_dir}/${elf_base}.resources.txt"
    "loadable_flash_bytes=${flash_bytes}\nflash_gate_bytes=${flash_gate}\nstatic_ram_gate_bytes=${static_gate}\nstack_reserved_bytes=4096\nminimum_uncommitted_ram_bytes=${uncommitted_gate}\n")
message(STATUS "Loadable firmware: ${flash_bytes}/${flash_gate} bytes; linker SRAM/stack gates passed")

execute_process(
    COMMAND "${ARM_SIZE}" "${ELF_FILE}"
    OUTPUT_FILE "${SIZE_FILE}"
    ERROR_FILE "${SIZE_FILE}"
    RESULT_VARIABLE size_result
)

if(NOT size_result EQUAL 0)
    message(FATAL_ERROR "arm-none-eabi-size failed with code ${size_result}")
endif()
