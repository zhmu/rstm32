set(CMAKE_CROSSCOMPILING TRUE)
SET(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(TARGET arm-none-eabi)

set(CPU STM32F1)

SET(CMAKE_C_COMPILER_TARGET ${TARGET})
SET(CMAKE_C_COMPILER ${TARGET}-gcc)
SET(CMAKE_CXX_COMPILER_TARGET ${TARGET})
SET(CMAKE_CXX_COMPILER ${TARGET}-g++)
SET(CMAKE_ASM_COMPILER_TARGET ${TARGET})
SET(CMAKE_OBJCOPY ${TARGET}-objcopy)

set(GENERIC_FLAGS "-mcpu=cortex-m3 -mthumb -msoft-float -mfix-cortex-m3-ldrd -ggdb3")
set(GENERIC_FLAGS "${GENERIC_FLAGS} -Os -Wall -Wextra -Wshadow -Wredundant-decls -fno-common -ffunction-sections -fdata-sections -Wundef -DCPU_${CPU}")
set(GENERIC_FLAGS "${GENERIC_FLAGS} -Os -DNDEBUG")
set(CMAKE_C_FLAGS ${GENERIC_FLAGS})
set(CMAKE_CXX_FLAGS "${GENERIC_FLAGS} -std=c++2a -fno-rtti -fno-exceptions")

set(CMAKE_EXE_LINKER_FLAGS "--static -nostartfiles -Wl,--gc-sections -T ${CMAKE_CURRENT_LIST_DIR}/bluepill.ld -Wl,-Map=${PROJECT_NAME}.map")
