##
## Make and program TinyFPGA BX
##

BASENAME = tinydfu
TARGETNAME = $(BASENAME)_tbx
PROJTOP = $(TARGETNAME)

RTL_USB_DIR = ../../usb

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
	$(RTL_USB_DIR)/usb_string_rom.v \
	$(RTL_USB_DIR)/usb_dfu_ctrl_ep.v \
	$(RTL_USB_DIR)/usb_spiflash_bridge.v \
	$(RTL_USB_DIR)/usb_dfu_core.v \
	$(RTL_USB_DIR)/usb_phy_ice40.v \
	pll.v

SRC = $(PROJTOP).v $(SOURCES)

PIN_DEF = pins.pcf

DEVICE = lp8k
PACKAGE = cm81

CLK_MHZ = 48

all: $(PROJTOP).rpt $(PROJTOP).bin $(PROJTOP).hex

pll.v:
	icepll -i 16 -o $(CLK_MHZ) -m -f $@

synth: $(PROJTOP).json

$(PROJTOP).json: $(SRC)
	yosys -q -p 'synth_ice40 -top $(PROJTOP) -json $@' $^

%.asc: $(PIN_DEF) %.json
	nextpnr-ice40 -r --$(DEVICE) --opt-timing --package $(PACKAGE) --pcf $(PIN_DEF) --json $*.json --asc $@ --pre-pack constraints.py

gui: $(PIN_DEF) $(PROJTOP).json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --pcf $(PIN_DEF) --json $(PROJTOP).json --asc $(PROJTOP).asc --gui

%.bin: %.asc
	icepack $< $(@:.bin=-boot.bin)
	cp $(@:.bin=-boot.bin) $(@:.bin=-user.bin)
	icemulti -v -o $(@:.bin=-multi.bin) -a15 -p0 $(@:.bin=-boot.bin) $(@:.bin=-user.bin)
	(dd if=$(@:.bin=-multi.bin) bs=160 count=1 && cat $(@:.bin=-boot.bin)) > $@

%.rpt: %.asc
	icetime -d $(DEVICE) -mtr $@ $<

%.hex: %.bin
	hexdump $^ > $@

prog: $(PROJTOP).bin
	tinyprog -p $<

clean:
	rm -f $(PROJTOP).json $(PROJTOP).asc $(PROJTOP).rpt $(PROJTOP).bin $(PROJTOP)-*.bin pll.v

.SECONDARY:
.PHONY: all synth prog clean gui
