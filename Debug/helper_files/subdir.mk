################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../helper_files/helper_files.c 

C_DEPS += \
./helper_files/helper_files.d 

OBJS += \
./helper_files/helper_files.o 


# Each subdirectory must supply rules for building sources it contributes
helper_files/%.o: ../helper_files/%.c helper_files/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -std=c11 -I"C:\Users\Barna\eclipse-workspace\C_Kalman_filter_dev\data" -I"C:\Users\Barna\eclipse-workspace\C_Kalman_filter_dev\helper_files" -I"C:\Users\Barna\eclipse-workspace\C_Kalman_filter_dev\inc" -I"C:\Users\Barna\eclipse-workspace\C_Kalman_filter_dev\src" -O2 -g -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-helper_files

clean-helper_files:
	-$(RM) ./helper_files/helper_files.d ./helper_files/helper_files.o

.PHONY: clean-helper_files

