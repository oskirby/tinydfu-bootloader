module usb_string_rom #(
    // The default language is US English.
    parameter LANGID = 16'h0409,
    // The list of strings, with explicit NULL termination.
    parameter STRINGS = {
        "Example Board", 8'h00,
        "ACME Systems", 8'h00
    }
) (
    input [7:0] str_index,
    input [7:0] rom_addr,
    output [7:0] rom_length,
    output [7:0] rom_data
);

// The string ROM contains 2 bytes for each character, plus
// an extra 4 bytes for the language descriptor at index zero.
localparam STRING_CHARS = $size(STRINGS) / 8; 
localparam STRING_ROM_SIZE = (STRING_CHARS * 2) + 4;
localparam STRING_ROM_BITS = $clog2(STRING_ROM_SIZE);

// Extract a single character from the string, at a given byte offset.
// This is written in a very counter-intuitive way, because Vivado
// is unable to identify a slicing range within a function as constant.
function [7:0] get_str_char;
    input integer coff;
    reg [7:0] cval;
    begin
        cval[0] = STRINGS[$size(STRINGS) - coff*8 - 8];
        cval[1] = STRINGS[$size(STRINGS) - coff*8 - 7];
        cval[2] = STRINGS[$size(STRINGS) - coff*8 - 6];
        cval[3] = STRINGS[$size(STRINGS) - coff*8 - 5];
        cval[4] = STRINGS[$size(STRINGS) - coff*8 - 4];
        cval[5] = STRINGS[$size(STRINGS) - coff*8 - 3];
        cval[6] = STRINGS[$size(STRINGS) - coff*8 - 2];
        cval[7] = STRINGS[$size(STRINGS) - coff*8 - 1];
    end
    get_str_char=cval;
endfunction

// Count the number of NULLs to determine how many string descriptors exist.
function [7:0] get_num_strings;
    integer nbyte;
    integer ncount;
    begin
        ncount = 0;
        for (nbyte=0; nbyte < STRING_CHARS; nbyte = nbyte + 1) begin
            if (get_str_char(nbyte) == 8'h00) ncount = ncount + 1;
        end
    end
    get_num_strings=ncount;
endfunction
generate
    localparam STRING_ROM_DESCS = get_num_strings() + 1;
endgenerate

// Locate the offset, in bytes, of the desired string.
function [$clog2(STRING_CHARS)-1:0] get_str_offset;
    input integer oindex;
    integer obyte;
    begin
        for (obyte=0; (obyte < STRING_CHARS) && (oindex > 1); obyte = obyte + 1) begin
            if (get_str_char(obyte) == 8'h00) oindex = oindex - 1;
        end
    end
    get_str_offset=obyte;
endfunction

// Return the length of the string, in bytes, starting at offset.
function [7:0] get_str_length;
    input integer loffset;
    integer lbyte;
    integer lcount;
    begin
        lcount = 0;
        for (lbyte=loffset; (lbyte < STRING_CHARS) && (get_str_char(lbyte) != 8'h00); lbyte = lbyte + 1) begin
            lcount = lcount + 1;
        end
    end
    get_str_length=lcount;
endfunction

// Build the string ROM and its index tables.
reg [7:0] str_rom[STRING_ROM_SIZE-1:0];
reg [7:0] str_rom_sizes[STRING_ROM_DESCS-1:0];
reg [STRING_ROM_BITS-1:0] str_rom_offsets[STRING_ROM_DESCS-1:0];
genvar index, j;
generate
    // Create the language descriptor.
    initial str_rom[0] <= 4;            // bLength
    initial str_rom[1] <= 3;            // bDescriptorType == STRING
    initial str_rom[2] <= LANGID >> 0;  // wLANGID[0]
    initial str_rom[3] <= LANGID >> 8;  // wLANGID[1]
    initial str_rom_offsets[0] <= 0;
    initial str_rom_sizes[0] <= 4;

    // Create individual string descriptors
    for (index = 1; index < STRING_ROM_DESCS; index = index + 1) begin
        localparam offset = get_str_offset(index);
        localparam length = get_str_length(offset);
        initial str_rom[(offset*2) + 4] <= (length * 2) + 2;
        initial str_rom[(offset*2) + 5] <= 8'h03;
        initial str_rom_offsets[index] <= ((offset*2) + 4);
        initial str_rom_sizes[index] <= ((length*2) + 2);
        //initial $display("string[%d] at %d", index, offset);
        //initial $display("string[%d] len %d", index, length);
        //initial $display("str_rom[%d] = %x", (offset*2) + 4, (length * 2) + 2);
        //initial $display("str_rom[%d] = %x", (offset*2) + 5, 8'h03);

        for (j = 0; j < length; j = j + 1) begin
            localparam [7:0] ch = get_str_char(offset+j);
            initial str_rom[(offset*2) + 6 + (j*2)] <= ch;
            initial str_rom[(offset*2) + 7 + (j*2)] <= 8'h00;
            //initial $display("str_rom[%d] = %x", (offset*2) + 6 + (j*2), ch);
            //initial $display("str_rom[%d] = %x", (offset*2) + 7 + (j*2), 8'h00);
        end
        //initial $display("");
    end
endgenerate

// Mux out the rom data.
wire [STRING_ROM_BITS-1:0] rom_offset;
assign rom_offset = str_rom_offsets[str_index];
assign rom_length = str_rom_sizes[str_index];
assign rom_data = str_rom[rom_offset + rom_addr];

endmodule
