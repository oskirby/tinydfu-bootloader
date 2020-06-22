# Cross-platform iCEstick build script

# Steven Herbst <sgherbst@gmail.com>
# Updated: February 28, 2017

# `python make.py build` will build 
# `python make.py upload` will upload the generated bitstream to the FPGA
# `python make.py clean` will remove generated binaries

# Inspired by CS448H, Winter 2017
# https://github.com/rdaly525/CS448H

import os
import sys
import argparse
from subprocess import call
from glob import glob

PROJTOP = 'logicbone_ecp5'
RTL_USB_DIR = "../../usb"
RTL_I2C_DIR = "../../i2c"

PIN_DEF = 'logicbone-rev0.lpf'
DEVICE = '--um5g-45k'
PACKAGE = 'CABGA381'

SOURCES = [PROJTOP+'.v',
           RTL_USB_DIR+"/edge_detect.v",
           RTL_USB_DIR+"/usb_fs_in_arb.v",
           RTL_USB_DIR+"/usb_fs_in_pe.v",
           RTL_USB_DIR+"/usb_fs_out_arb.v",
           RTL_USB_DIR+"/usb_fs_out_pe.v",
           RTL_USB_DIR+"/usb_fs_pe.v",
           RTL_USB_DIR+"/usb_fs_rx.v",
           RTL_USB_DIR+"/usb_fs_tx_mux.v",
           RTL_USB_DIR+"/usb_fs_tx.v",
           RTL_USB_DIR+"/usb_reset_det.v",
           RTL_USB_DIR+"/usb_dfu_ctrl_ep.v",
           RTL_USB_DIR+"/usb_spiflash_bridge.v",
           RTL_USB_DIR+"/usb_dfu_core.v",
           RTL_USB_DIR+"/usb_dfu_ecp5.v",
           RTL_I2C_DIR+"/i2c_master.v",
           "logicbone_pll.v"]

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('command', nargs='*', default=['build'], help='build|upload|clean')
    args = parser.parse_args()

    # Implement case insensitive commands
    commands = args.command

    # Name of the Verilog source file *without* the ".v" extension
    name = PROJTOP
    
    # Platform-specific path to IceStorm tools
    if os.name=='nt':
        pio_rel = '.apio\\packages'
        #pio_rel = '.platformio\\packages\\toolchain-icestorm\\bin'
        home_path = os.getenv('HOMEPATH')

        # Build the full path to IceStorm tools
        pio = os.path.join(home_path, pio_rel)

        openocd_path = 'C:\\Data\\Programs\\openocd-20200530\\OpenOCD-20200530-0.10.0\\bin'
        
        # Tools used in the flow
        ecppack       = os.path.join(pio, 'toolchain-ecp5\\bin\\ecppack.exe')
        nextpnr_ecp5  = os.path.join(pio, 'toolchain-ecp5\\bin\\nextpnr-ecp5.exe')
        yosys         = os.path.join(pio, 'toolchain-yosys\\bin\\yosys.exe')
        openocd       = os.path.join(openocd_path, 'openocd.exe')

    else:
        pio_rel = '.platformio/packages/toolchain-icestorm/bin'
        home_path = os.environ['HOME']
        file_ext = ''
    
        # Build the full path to IceStorm tools
        pio = os.path.join(home_path, pio_rel)
        
        # Tools used in the flow
        ecppack       = os.path.join(pio, 'icepack'+file_ext)
        nextpnr_ecp5  = os.path.join(pio, 'nextpnr-ecp5'+file_ext)
        yosys         = os.path.join(pio, 'yosys'+file_ext)

    sources = SOURCES #glob('*.v')
    #sources += glob('./usb/*.v')

    for command in commands:
        # run command
        if command == 'build':
            synth_cmd = 'synth_ecp5 -top '+PROJTOP+' -json '+name+'.json'
            if call([yosys, '-q', '-p', synth_cmd] + sources) != 0:
                return
            if call([nextpnr_ecp5, '--json', name+'.json', '--textcfg', name+'_out.config', DEVICE, '--package', PACKAGE, '--lpf', PIN_DEF]) != 0:
                return
            if call([ecppack, '--svf', name+'.svf', name+'_out.config']) != 0:
                return
            if call([ecppack, '--compress', '--spimode', 'qspi', name+'_out.config', name+'.bit']) != 0:
                return
            
        elif command == 'upload':
            # openocd -f logicbone-jlink.cfg -c "transport select jtag; init; svf $<; exit"
            if call([openocd, '-f', 'logicbone-jlink-windows.cfg']) != 0:
                return
        
        elif command == 'clean':
            del_files = glob('*.bin')+glob('*.blif')+glob('*.rpt')+glob('*.asc') + [name+'.json'] + [name+'_out.config']
            for del_file in del_files:
                os.remove(del_file)
            
        else:
            raise Exception('Invalid command')

if __name__=='__main__':
    main()
