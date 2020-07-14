arty-a7-35t:
	vivado -mode batch -source bootloader.tcl -tclargs xc7a35tcsg324-1

arty-a7-100t:
	vivado -mode batch -source bootloader.tcl -tclargs xc7a100tcsg324-1

# Program the FPGA using OpenOCD
prog: bootloader.bit
	openocd -f openocd_xc7_ft2232.cfg -c "init" -c "pld load 0 $<" -c "exit"

dfu: bootloader.bit
	dfu-util -d 1d50:6130 -a2 -D $<

# Delete Vivado-generated content
clean:
	rm -vf *.bit *.dcp *.log *.jou *.prm *.xpr *.rpt
	rm -vrf *.cache/ *.hw/ *.srcs/
	rm -vrf *.ip_user_files/

.PHONY: all clean prog

