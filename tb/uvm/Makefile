QUESTA_HOME ?= /data1/questaone_sim/questasim
QUESTA_PLATFORM ?= linux_x86_64
ETH_LIC_SERVER := 8161@lic-mentor.ethz.ch

export LM_LICENSE_FILE := $(ETH_LIC_SERVER)
export MGLS_LICENSE_FILE := $(LM_LICENSE_FILE)
export SALT_LICENSE_SERVER := $(LM_LICENSE_FILE)
export QSIM_INI := $(QUESTA_HOME)/modelsim.ini

QUESTA_BIN := $(QUESTA_HOME)/$(QUESTA_PLATFORM)
VLIB := $(QUESTA_BIN)/vlib
VMAP := $(QUESTA_BIN)/vmap
VCOM := $(QUESTA_BIN)/vcom
VLOG := $(QUESTA_BIN)/vlog
VSIM := $(QUESTA_BIN)/vsim

BUILD_DIR ?= builds/default
WORK := $(BUILD_DIR)/work
ALTERA_MF := $(BUILD_DIR)/altera_mf

RTL_DIR := ../../rtl
TB_TOP := tb_onewire_controller_smoke
CASE ?= OW_SMOKE
MODE ?= isolated
VSIM_PLUSARGS ?= +OW_CASE=$(CASE) +OW_MODE=$(MODE)

.PHONY: smoke compile run regress clean

smoke: compile run

$(WORK):
	mkdir -p $(BUILD_DIR)
	$(VLIB) $(WORK)

$(ALTERA_MF):
	mkdir -p $(BUILD_DIR)
	$(VLIB) $(ALTERA_MF)

compile: $(WORK) $(ALTERA_MF)
	$(VMAP) work $(WORK)
	$(VMAP) altera_mf $(ALTERA_MF)
	$(VCOM) -2008 -work work harness/models/altera_mf/scfifo_sim.vhd
	$(VCOM) -2008 -work work $(RTL_DIR)/common/pseudo_clock_down_convertor.vhd
	$(VCOM) -2008 -work work $(RTL_DIR)/vhdl/onewire_master/onewire_master.vhd
	$(VCOM) -2008 -work work $(RTL_DIR)/vhdl/onewire_master_controller/onewire_master_controller.vhd
	$(VLOG) -sv -work work harness/models/ds18b20/onewire_rc_line_model.sv harness/models/ds18b20/ds18b20_1wire_model.sv tb_onewire_controller_smoke.sv

run:
	$(VSIM) -c -suppress 19 -suppress 3009 -work work $(TB_TOP) $(VSIM_PLUSARGS) -do "run -all; quit -f"

regress: compile
	python3 script/run_onewire_regression.py --skip-compile

clean:
	rm -rf $(BUILD_DIR) transcript vsim.wlf
