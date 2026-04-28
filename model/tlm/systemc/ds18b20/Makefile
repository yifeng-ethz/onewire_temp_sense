SYSTEMC_HOME ?= $(HOME)/.cache/codex/systemc-3.0.2
SYSTEMC_INC ?= $(SYSTEMC_HOME)/include
SYSTEMC_LIBDIR ?= $(SYSTEMC_HOME)/lib64
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -I$(SYSTEMC_INC)
LDFLAGS ?= -L$(SYSTEMC_LIBDIR) -lsystemc -pthread

BUILD_DIR ?= build
ARTIFACT_DIR ?= ../../../artifacts
TARGET := $(BUILD_DIR)/ds18b20_systemc_model
TRACE_CSV := $(ARTIFACT_DIR)/ds18b20_systemc_trace.csv

.PHONY: all run clean check-systemc

all: $(TARGET)

check-systemc:
	@test -f "$(SYSTEMC_INC)/systemc" || \
	  (echo "Missing SystemC headers under $(SYSTEMC_INC). Build Accellera SystemC and set SYSTEMC_HOME." >&2; exit 2)
	@test -f "$(SYSTEMC_LIBDIR)/libsystemc.a" -o -f "$(SYSTEMC_LIBDIR)/libsystemc.so" || \
	  (echo "Missing libsystemc under $(SYSTEMC_LIBDIR). Build Accellera SystemC and set SYSTEMC_HOME." >&2; exit 2)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(ARTIFACT_DIR):
	mkdir -p $(ARTIFACT_DIR)

$(TARGET): ds18b20_systemc_model.cpp | $(BUILD_DIR) check-systemc
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

run: $(TARGET) | $(ARTIFACT_DIR)
	DS18B20_TRACE_OUT=$(TRACE_CSV) $(TARGET)

clean:
	rm -rf $(BUILD_DIR)
