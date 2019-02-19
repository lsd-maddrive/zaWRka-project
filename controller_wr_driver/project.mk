ROSLIB = ./ros_lib
include $(ROSLIB)/ros.mk

PROJECT_MODULES = src/lld_control.c        		\
				  src/remote_control.c			\
				  src/lld_encoder.c				\
				  src/lld_odometry.c			\
				  
PROJECT_TESTS   = tests/test_lld_control.c     	\
				  tests/test_remote_control.c	\
				  tests/test_lld_encoder.c		\
				  tests/test_lld_odometry.c		\
				  		
PROJECT_CSRC    = src/main.c src/common.c src/debug.c \
    				$(PROJECT_MODULES) $(PROJECT_TESTS)

PROJECT_CPPSRC 	= $(ROSSRC) src/ros.cpp

PROJECT_INCDIR	= include tests $(ROSINC)

PROJECT_LIBS	= -lm

PROJECT_OPTS	= -specs=nosys.specs

