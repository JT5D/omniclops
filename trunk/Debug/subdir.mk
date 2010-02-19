################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../anyoption.cpp \
../bitmap.cpp \
../detectfloor.cpp \
../drawing.cpp \
../fast.cpp \
../libcam.cpp \
../main.cpp \
../motionmodel.cpp \
../omni.cpp \
../pointcloud.cpp \
../polynomial.cpp 

OBJS += \
./anyoption.o \
./bitmap.o \
./detectfloor.o \
./drawing.o \
./fast.o \
./libcam.o \
./main.o \
./motionmodel.o \
./omni.o \
./pointcloud.o \
./polynomial.o 

CPP_DEPS += \
./anyoption.d \
./bitmap.d \
./detectfloor.d \
./drawing.d \
./fast.d \
./libcam.d \
./main.d \
./motionmodel.d \
./omni.d \
./pointcloud.d \
./polynomial.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -I/usr/include/gstreamer-0.10 -O3 -g3 -Wall -c -fmessage-length=0 -lcam -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs gstreamer-0.10` -L/usr/local/lib -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


