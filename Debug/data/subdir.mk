################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../data/matrix_data.c 

C_DEPS += \
./data/matrix_data.d 

OBJS += \
./data/matrix_data.o 


# Each subdirectory must supply rules for building sources it contributes
data/%.o: ../data/%.c data/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -std=c11 -I"C:\Users\bteme\eclipse-workspace\C_Kalman_filter_dev\data" -I"C:\Users\bteme\eclipse-workspace\C_Kalman_filter_dev\kalman_filter" -I"C:\Users\bteme\eclipse-workspace\C_Kalman_filter_dev\helper_files" -I"C:\Users\bteme\eclipse-workspace\C_Kalman_filter_dev\inc" -I"C:\Users\bteme\eclipse-workspace\C_Kalman_filter_dev\src" -O2 -g -pedantic -Wall -Wextra -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-data

clean-data:
	-$(RM) ./data/matrix_data.d ./data/matrix_data.o

.PHONY: clean-data

