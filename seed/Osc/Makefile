# Project Name
TARGET = Osc

# Sources
CPP_SOURCES = Osc.cpp

# Library Locations
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP
LDFLAGS += -u _printf_float
CPP_STANDARD ?= -std=gnu++17
OPT ?= -O3

# Core location, and generic makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

