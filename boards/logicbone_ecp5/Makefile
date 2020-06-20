##
## Make and program TinyFPGA BX
##

BASENAME = tinydfu
TARGETNAME = $(BASENAME)_lbone
PROJTOP = logicbone_ecp5

RTL_USB_DIR = ../../usb

SOURCES = \
	$(RTL_USB_DIR)/edge_detect.v \
	$(RTL_USB_DIR)/serial.v \
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
	$(RTL_USB_DIR)/usb_dfu_ecp5.v \
	logicbone_pll.v

SRC = $(PROJTOP).v $(SOURCES)

PIN_DEF = logicbone-rev0.lpf

DEVICE = --um5g-45k
#DEVICE = --um5g-85k
PACKAGE = CABGA381

all: $(PROJTOP).bit $(PROJTOP).hex $(PROJTOP).svf

synth: $(PROJTOP).json

$(PROJTOP).json: $(SRC)
	yosys -q -p 'synth_ecp5 -top $(PROJTOP) -json $@' $^

%_out.config: %.json
	nextpnr-ecp5 --json $< --textcfg $@ $(DEVICE) --package $(PACKAGE) --lpf $(PIN_DEF)

%.svf: %_out.config
	ecppack --svf $@ $<

%.bit: %_out.config
	ecppack --compress --spimode qspi $< $@

%.hex: %.bit
	hexdump $^ > $@

prog: $(PROJTOP).svf
	openocd -f logicbone-jlink.cfg -c "transport select jtag; init; svf $<; exit"

clean:
	rm -f $(PROJTOP).json $(PROJTOP).svf $(PROJTOP).bit $(PROJTOP)_out.config $(PROJTOP).hex

.SECONDARY:
.PHONY: all synth prog clean gui
