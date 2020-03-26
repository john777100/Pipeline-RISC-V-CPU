To execute baseline:
	run "ncverilog Final_tb.v CHIP.v slow_memory.v +access+r +define+TestBenchName"

To execute BrPred on 2bit FSM:
	run "ncverilog Final_tb.v CHIP_BP_2bit.v slow_memory.v +access+r +define+BrPred"
To execute BrPred on 2AL:
	run "ncverilog Final_tb.v CHIP_BP_2AL.v slow_memory.v +access+r +define+BrPred"

To execute Compression:
	run "ncverilog Final_tb.v CHIP_final_compression_v1.v slow_memory.v +access+r +define+TestBenchName"

For gate-level simulation:
	change the .sdf in Final_tb.v to the corresponding one in syn/
	in the command, change the CHIP_*.v file to the corresponding .v file in syn/
	add "+define+SDF" at the end of the command
