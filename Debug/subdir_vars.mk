################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../empty.syscfg 

C_SRCS += \
../debug_uart.c \
../empty.c \
./ti_msp_dl_config.c \
D:/CCS/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../gimbal.c \
../gyro_link.c \
../vision_link.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./debug_uart.d \
./empty.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./gimbal.d \
./gyro_link.d \
./vision_link.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./debug_uart.o \
./empty.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./gimbal.o \
./gyro_link.o \
./vision_link.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"debug_uart.o" \
"empty.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"gimbal.o" \
"gyro_link.o" \
"vision_link.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"debug_uart.d" \
"empty.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"gimbal.d" \
"gyro_link.d" \
"vision_link.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../debug_uart.c" \
"../empty.c" \
"./ti_msp_dl_config.c" \
"D:/CCS/mspm0_sdk_2_11_00_07/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../gimbal.c" \
"../gyro_link.c" \
"../vision_link.c" 

SYSCFG_SRCS__QUOTED += \
"../empty.syscfg" 


