set RTL_USB_DIR ../../usb
set PROJECT bootloader

# Create Project

create_project -force -name ${PROJECT} -part xc7a100tcsg324-1
set_msg_config -id {Common 17-55} -new_severity {Warning}

# Add Sources

read_verilog -sv ${RTL_USB_DIR}/edge_detect.v
read_verilog -sv ${RTL_USB_DIR}/strobe.v
read_verilog -sv ${RTL_USB_DIR}/usb_fs_in_arb.v
read_verilog -sv ${RTL_USB_DIR}/usb_fs_in_pe.v
read_verilog -sv ${RTL_USB_DIR}/usb_fs_out_arb.v
read_verilog -sv ${RTL_USB_DIR}/usb_fs_out_pe.v
read_verilog -sv ${RTL_USB_DIR}/usb_fs_pe.v
read_verilog -sv ${RTL_USB_DIR}/usb_fs_rx.v
read_verilog -sv ${RTL_USB_DIR}/usb_fs_tx_mux.v
read_verilog -sv ${RTL_USB_DIR}/usb_fs_tx.v
read_verilog -sv ${RTL_USB_DIR}/usb_reset_det.v
read_verilog -sv ${RTL_USB_DIR}/usb_dfu_core.v
read_verilog -sv ${RTL_USB_DIR}/usb_dfu_ctrl_ep.v
read_verilog -sv ${RTL_USB_DIR}/usb_spiflash_bridge.v
read_verilog -sv ${RTL_USB_DIR}/usb_phy_xc7.v

read_verilog wbicapetwo.v
read_verilog ${PROJECT}.v

# Add EDIFs


# Add IPs

set PLLNAME usb_pll_clkwiz

create_ip -name clk_wiz -vendor xilinx.com -library ip -module_name ${PLLNAME}
set_property -dict [ eval list CONFIG.PRIM_IN_FREQ {100.000} \
	CONFIG.NUM_OUT_CLKS {1} \
	CONFIG.RESET_TYPE {ACTIVE_HIGH} \
	CONFIG.RESET_PORT {reset} \
	CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {48.000}] [get_ips ${PLLNAME}]
generate_target all [get_files ${PROJECT}.srcs/sources_1/ip/${PLLNAME}/${PLLNAME}.xci]
create_ip_run [get_ips ${PLLNAME}]
read_verilog ${PROJECT}.srcs/sources_1/ip/${PLLNAME}/${PLLNAME}_clk_wiz.v
read_verilog ${PROJECT}.srcs/sources_1/ip/${PLLNAME}/${PLLNAME}.v

# Add constraints

read_xdc arty_a7.xdc
set_property PROCESSING_ORDER EARLY [get_files arty_a7.xdc]

# Add pre-synthesis commands


# Synthesis

synth_design -directive default -top ${PROJECT} -part xc7a100tcsg324-1

# Synthesis report

report_timing_summary -file top_timing_synth.rpt
report_utilization -hierarchical -file top_utilization_hierarchical_synth.rpt
report_utilization -file top_utilization_synth.rpt

# Optimize design

opt_design -directive default

# Add pre-placement commands


# Placement

place_design -directive default

# Placement report

report_utilization -hierarchical -file top_utilization_hierarchical_place.rpt
report_utilization -file top_utilization_place.rpt
report_io -file top_io.rpt
report_control_sets -verbose -file top_control_sets.rpt
report_clock_utilization -file top_clock_utilization.rpt

# Add pre-routing commands


# Routing

route_design -directive default
phys_opt_design -directive default
write_checkpoint -force top_route.dcp

# Routing report

report_timing_summary -no_header -no_detailed_paths
report_route_status -file top_route_status.rpt
report_drc -file top_drc.rpt
report_timing_summary -datasheet -max_paths 10 -file top_timing.rpt
report_power -file top_power.rpt
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]

# Bitstream generation

write_bitstream -force ${PROJECT}.bit 
write_cfgmem -force -format bin -interface spix4 -size 16 -loadbit "up 0x0 ${PROJECT}.bit" -file ${PROJECT}.bin

# End

quit
