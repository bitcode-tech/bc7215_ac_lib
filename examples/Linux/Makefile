# Linux port Makefile for bc7215_ac_lib.
# Place this directory at: examples/Linux inside the bc7215_ac_lib repository.

ROOT ?= ../..
CORE ?= $(ROOT)/bc7215_ac_lib

CC := $(CROSS_COMPILE)gcc
CXX := $(CROSS_COMPILE)g++
AR := $(CROSS_COMPILE)ar

TARGET ?= bc7215_ac_demo
BUILD_DIR ?= build

CPPFLAGS += -Iinclude -I$(CORE)
CFLAGS += -std=c99 -Wall -Wextra -O2
CXXFLAGS += -std=c++11 -Wall -Wextra -O2
LDFLAGS += -pthread

C_SRCS := \
    $(CORE)/bc7215_lib.c \
    $(CORE)/bc7215_ac_lib.c

CXX_SRCS := \
    src/bc7215.cpp \
    src/bc7215ac.cpp \
    src/console_line.cpp \
    src/main.cpp

# Keep all generated files inside examples/Linux/build.
# Do not derive object paths directly from ../.. source paths, otherwise make may
# create object files outside this directory, for example ../../bc7215_ac_lib/*.o.
C_OBJS := $(patsubst $(CORE)/%.c,$(BUILD_DIR)/core/%.o,$(C_SRCS))
CXX_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/src/%.o,$(CXX_SRCS))
OBJS := $(C_OBJS) $(CXX_OBJS)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/core/%.o: $(CORE)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/src/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

.PHONY: all clean
