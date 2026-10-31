# Toolchain description for Arm GNU cross compilation.
#
# The repository-local hint is the locked GNU Arm toolchain used for the
# reviewed STM32C071 diagnostic. A normal PATH installation remains supported.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(SIMPLECLOCK_ARM_GNU_BIN
    "${CMAKE_CURRENT_LIST_DIR}/../.tools/arm-gnu-toolchain-14.3.rel1/bin")

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc
    HINTS "${SIMPLECLOCK_ARM_GNU_BIN}" REQUIRED)
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc
    HINTS "${SIMPLECLOCK_ARM_GNU_BIN}" REQUIRED)
find_program(CMAKE_AR arm-none-eabi-ar
    HINTS "${SIMPLECLOCK_ARM_GNU_BIN}" REQUIRED)
find_program(CMAKE_RANLIB arm-none-eabi-ranlib
    HINTS "${SIMPLECLOCK_ARM_GNU_BIN}" REQUIRED)
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy
    HINTS "${SIMPLECLOCK_ARM_GNU_BIN}" REQUIRED)
find_program(CMAKE_SIZE arm-none-eabi-size
    HINTS "${SIMPLECLOCK_ARM_GNU_BIN}" REQUIRED)

set(CMAKE_C_FLAGS_INIT "-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft")
set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft")
