cmake_minimum_required(VERSION 3.8)

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
set (PROJECT_NAME             "inmp441")
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
	Core/Src/i2s.c
	Core/Src/main.c
	Core/Src/gpio.c
	Core/Src/dma.c
	Core/Src/stm32f4xx_hal_msp.c
	Core/Src/stm32f4xx_it.c
	Core/Src/sysmem.c
	Core/Src/system_stm32f4xx.c
)

set(SRC_USBD
	Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
	USB_DEVICE/App/usb_device.c
	USB_DEVICE/App/usbd_cdc_if.c
	USB_DEVICE/App/usbd_desc.c
	USB_DEVICE/Target/usbd_conf.c
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
	Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
)

file(GLOB SRC_DRIVER
	Drivers/STM32F4xx_HAL_Driver/Src/*.c
)

set(SOURCES
	${SRC_CORE}
	${SRC_DRIVER}
	# ${SRC_USBD}
)

list(APPEND PROJECT_DEFINES
	# LIST COMPILER DEFINITIONS HERE
	STM32F407xx
	USE_HAL_DRIVER
)

list(APPEND PROJECT_INCLUDE_PATH
	# LIST INCLUDE DIRECTORIES HERE
	Core/Inc
	Drivers/STM32F4xx_HAL_Driver/Inc
	Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
	Drivers/CMSIS/Include
	Drivers/CMSIS/Device/ST/STM32F4xx/Include
	Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
	Middlewares/ST/STM32_USB_Device_Library/Core/Inc
	USB_DEVICE/App
	USB_DEVICE/Target
)
############ MODIFY ACCORDING TO REQUIREMENTS) ########################

#######################################################################
set(CMAKE_TOOLCHAIN_FILE "gcc-arm-none-eabi.cmake")
################## PROJECT SETUP ######################################
project(${PROJECT_NAME} C CXX ASM)

add_executable(${PROJECT_NAME} ${SOURCES})
add_compile_definitions(${PROJECT_DEFINES})
include_directories(${PROJECT_INCLUDE_PATH})

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)

set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
	set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -O3")
else()
	set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -Og -g3 -ggdb")
endif()
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=${PROJECT_NAME}.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
