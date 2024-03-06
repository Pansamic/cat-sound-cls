cmake_minimum_required(VERSION 3.20)

###################### CONSTANTS ######################################
set (PROJECT_TYPE_EXECUTABLE          "exe")
set (PROJECT_TYPE_STATIC_LIBRARY      "static-lib")
set (MCPU_CORTEX_M0				      "-mcpu=cortex-m0")
set (MCPU_CORTEX_M0PLUS				  "-mcpu=cortex-m0plus")
set (MCPU_CORTEX_M3				      "-mcpu=cortex-m3")
set (MCPU_CORTEX_M4				      "-mcpu=cortex-m4")
set (MCPU_CORTEX_M7				      "-mcpu=cortex-m7")
set (MCPU_CORTEX_M33				  "-mcpu=cortex-m33")
set (MFPU_FPV4_SP_D16                 "-mfpu=fpv4-sp-d16")
set (MFPU_FPV5_D16                    "-mfpu=fpv5-d16")
set (RUNTIME_LIBRARY_REDUCED_C        "--specs=nano.specs")
set (RUNTIME_LIBRARY_STD_C            "")
set (RUNTIME_LIBRARY_SYSCALLS_MINIMAL "--specs=nosys.specs")
set (RUNTIME_LIBRARY_SYSCALLS_NONE    "")
set (MFLOAT_ABI_SOFTWARE              "-mfloat-abi=soft")
set (MFLOAT_ABI_HARDWARE              "-mfloat-abi=hard")
set (MFLOAT_ABI_MIX                   "-mfloat-abi=softfp")
#######################################################################

###################### VARIABLES ######################################
set (PROJECT_NAME             "SoundClass")
set (PROJECT_TYPE             "exe")
set (LINKER_SCRIPT            "../STM32F407ZGTX_FLASH.ld")
set (MCPU                     ${MCPU_CORTEX_M4})
set (MFPU                     ${MFPU_FPV4_SP_D16})
set (MFLOAT_ABI               ${MFLOAT_ABI_HARDWARE})
set (RUNTIME_LIBRARY          ${RUNTIME_LIBRARY_REDUCED_C})
set (RUNTIME_LIBRARY_SYSCALLS ${RUNTIME_LIBRARY_SYSCALLS_MINIMAL})

set(PROJECT_INCLUDE_PATH "" CACHE PATH "Path to project include files")
set(PROJECT_SOURCE_FILES "" CACHE FILEPATH "Path to project source files")
set(PROJECT_DEFINES "" CACHE STRING "Project defines")

set(SRC_CORE
	Core/Startup/startup_stm32f407zgtx.s
	Core/Src/mfcc.c
	Core/Src/crc.c
	Core/Src/dma.c
	Core/Src/fsmc.c
	Core/Src/gpio.c
	Core/Src/i2s.c
	Core/Src/st7789.c
	Core/Src/main.c
	Core/Src/spi.c
	Core/Src/stm32f4xx_hal_msp.c
	Core/Src/stm32f4xx_it.c
	Core/Src/sysmem.c
	Core/Src/system_stm32f4xx.c
	Core/Src/w25qxx.c
	Core/Src/ppbuf.c
)

set(SRC_AI
	X-CUBE-AI/App/cat_sound_cls_data_params.c
	X-CUBE-AI/App/cat_sound_cls_data.c
	X-CUBE-AI/App/cat_sound_cls.c
)

set(SRC_DRIVER
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c
)

set(PROJECT_DEFINES
	# LIST COMPILER DEFINITIONS HERE
	STM32F407xx
	USE_HAL_DRIVER
	ARM_MATH_CM4
)

set(PROJECT_INCLUDE_PATH
	# LIST INCLUDE DIRECTORIES HERE
	Core/Inc
	Drivers/STM32F4xx_HAL_Driver/Inc
	Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
	Drivers/CMSIS/Include
	Drivers/CMSIS/Device/ST/STM32F4xx/Include
	Middlewares/ST/ARM/DSP/Inc
	Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
	Middlewares/ST/STM32_USB_Device_Library/Core/Inc
	Middlewares/ST/AI/Inc
	USB_DEVICE/App
	USB_DEVICE/Target
	X-CUBE-AI/App
	X-CUBE-AI/Target
)
############ MODIFY ACCORDING TO REQUIREMENTS) ########################

#######################################################################
set(CMAKE_TOOLCHAIN_FILE "gcc-arm-none-eabi.cmake")
################## PROJECT SETUP ######################################
project(${PROJECT_NAME} C CXX ASM)

add_executable(${PROJECT_NAME} ${SRC_CORE} ${SRC_AI} ${SRC_DRIVER})
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)

add_compile_definitions(${PROJECT_DEFINES})
include_directories(${PROJECT_INCLUDE_PATH})

# link_directories(${PROJECT_LINKER_LIB_PATH})
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/Middlewares/ST/AI/Lib/NetworkRuntime810_CM4_GCC.a)
target_link_libraries(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a)

set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
	set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -O3")
else()
	set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -Og -g3 -ggdb")
endif()
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
