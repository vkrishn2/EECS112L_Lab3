OUTPUT = test
all: $(OUTPUT)

test:
	vlog -64 -sv -f rtl.cfg
	vlog -64 -sv -f tb.cfg -work work
	vopt -64 tb_top -o tb_top_opt +acc -work work
	vsim -debugDB -64 -c tb_top_opt -do sim.do
	vsim -debugDB -64 -gui -view waveform.wlf
