/* DFU Board information definitions for the Digilent Arty-A7 */
localparam SPI_FLASH_SIZE = (16 * 1024 *1024);
localparam SPI_PAGE_SIZE  = 256;

/* Flash partition layout */
localparam BOOTPART_SIZE = (2 * 1024 * 1024);
localparam USERPART_SIZE = (2 * 1024 * 1024);
localparam DATAPART_SIZE = (SPI_FLASH_SIZE - BOOTPART_SIZE - USERPART_SIZE);

localparam BOOTPART_START = 0;
localparam USERPART_START = BOOTPART_START + BOOTPART_SIZE;
localparam DATAPART_START = USERPART_START + USERPART_SIZE;

/* How many security registers are there? */
localparam SPI_SECURITY_REGISTERS = 3;
localparam SPI_SECURITY_REG_SHIFT = 8;

/* USB VID/PID Definitions */
localparam BOARD_VID = 'h1d50;  /* OpenMoko Inc. */
localparam BOARD_PID = 'h6130;  /* TinyFPGA Bootloader */

/* String Descriptors */
localparam BOARD_MFR_NAME = "Digilent";
localparam BOARD_PRODUCT_NAME = "Arty-A7";
