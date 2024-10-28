################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/kalman_filter.c \
../src/main.c \
../src/matrix_compute.c 

C_DEPS += \
./src/kalman_filter.d \
./src/main.d \
./src/matrix_compute.d 

OBJS += \
./src/kalman_filter.o \
./src/main.o \
./src/matrix_compute.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -std=c11 -I"C:\Users\Barna\eclipse-workspace\C_Kalman_filter_dev\data" -I"C:\Users\Barna\eclipse-workspace\C_Kalman_filter_dev\helper_files" -I"C:\Users\Barna\eclipse-workspace\C_Kalman_filter_dev\inc" -I"C:\Users\Barna\eclipse-workspace\C_Kalman_filter_dev\src" -O2 -g -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-src

clean-src:
	-$(RM) ./src/kalman_filter.d ./src/kalman_filter.o ./src/main.d ./src/main.o ./src/matrix_compute.d ./src/matrix_compute.o

.PHONY: clean-src

