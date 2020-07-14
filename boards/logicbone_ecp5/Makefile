##
## Make and program TinyFPGA BX
##

BASENAME = tinydfu
TARGETNAME = $(BASENAME)_lbone
PROJTOP = logicbone_ecp5

RTL_USB_DIR = ../../usb
RTL_I2C_DIR = ../../i2c

SOURCES = \
	$(RTL_USB_DIR)/edge_detect.v \
	$(RTL_USB_DIR)/strobe.v \
	$(RTL_USB_DIR)/usb_fs_in_arb.v \
	$(RTL_USB_DIR)/usb_fs_in_pe.v \
	$(RTL_USB_DIR)/usb_fs_out_arb.v \
	$(RTL_USB_DIR)/usb_fs_out_pe.v \
	$(RTL_USB_DIR)/usb_fs_pe.v \
	$(RTL_USB_DIR)/usb_fs_rx.v \
	$(RTL_USB_DIR)/usb_fs_tx_mux.v \
	$(RTL_USB_DIR)/usb_fs_tx.v \
	$(RTL_USB_DIR)/usb_reset_det.v \
	$(RTL_USB_DIR)/usb_dfu_ctrl_ep.v \
	$(RTL_USB_DIR)/usb_spiflash_bridge.v \
	$(RTL_USB_DIR)/usb_dfu_core.v \
	$(RTL_USB_DIR)/usb_phy_ecp5.v \
	$(RTL_I2C_DIR)/i2c_master.v \
	i2c_bringup.v \
	logicbone_pll.v

SRC = $(PROJTOP).v $(SOURCES)

PIN_DEF = logicbone-rev0.lpf

DEVICE = --um5g-45k
#DEVICE = --um5g-85k
PACKAGE = CABGA381
BOOTADDR = 0x100000 # User image starts at 1MB

all: $(PROJTOP).bit $(PROJTOP).hex $(PROJTOP).svf
bootloader: $(PROJTOP).bit

synth: $(PROJTOP).json

$(PROJTOP).json: $(SRC)
	yosys -q -p 'synth_ecp5 -top $(PROJTOP) -json $@' $^

%_out.config: %.json
	nextpnr-ecp5 --json $< --textcfg $@ $(DEVICE) --package $(PACKAGE) --lpf $(PIN_DEF)

%.svf: %_out.config
	ecppack --svf $@ $<

%.bit: %_out.config
	ecppack --compress --bootaddr $(BOOTADDR) --spimode qspi $< $@

%.hex: %.bit
	hexdump $^ > $@

prog: $(PROJTOP).svf
	openocd -f logicbone-jlink.cfg -c "transport select jtag; init; svf $<; exit"

dfu: $(PROJTOP).bit
	dfu-util -d 1d50:615d -a2 -D $<

clean:
	rm -f $(PROJTOP).json $(PROJTOP).svf $(PROJTOP).bit $(PROJTOP)_out.config $(PROJTOP).hex

.SECONDARY:
.PHONY: all bitstream synth prog dfu clean gui
