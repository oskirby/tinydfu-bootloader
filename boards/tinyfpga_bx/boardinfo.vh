/* DFU Board information definitions for the Logicbone ECP5 */
localparam SPI_FLASH_SIZE = (1 * 1024 *1024);
localparam SPI_PAGE_SIZE  = 256;

/* Flash partition layout */
localparam BOOTPART_SIZE = (256 * 1024);
localparam USERPART_SIZE = (256 * 1024);
localparam DATAPART_SIZE = (SPI_FLASH_SIZE - BOOTPART_SIZE - USERPART_SIZE);

/* USB VID/PID Definitions */
localparam BOARD_VID = 'h1d50;  /* OpenMoko Inc. */
localparam BOARD_PID = 'h6130;  /* TinyFPGA Bootloader */
