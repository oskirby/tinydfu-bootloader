Ported to an Arty A7.
A boot pin is used to select the DFU bootloader or user image. 
The boot pin is connected to switch 1 in this example.
 
Used https://github.com/TomKeddie/TinyFPGA-Bootloader as reference for multiboot. 
If boot pin is made high, the "normal" image is booted, else, USB DFU.

usage: flash generated file to SPI flash.

Boot board

Plug in USB

Write an FW image to the user image section

Set boot pin to 1


dfu boot is located from 0x00000000 to 0x00200000

user image is located from 0x00200000 to 0x00400000

The other 12MB of flash is free for user data.

Vivado 2019.2 was used, other versions might not work as expected.
