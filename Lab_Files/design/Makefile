OUTPUT = test
all: $(OUTPUT)

test:
	vlog -64 -sv -f rtl.cfg +define+__SIM__
	vlog -64 -sv -f tb.cfg -work work
	vopt -64 tb_top -o tb_top_opt +acc -work work 
	vsim -debugDB -64 -c tb_top_opt -do sim.do +define+__SIM__
