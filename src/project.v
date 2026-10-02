`default_nettype none

module tt_um_toxicfox_microphoenix (
    input wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input wire ena,
    input wire clk,
    input wire rst_n
);
    wire reset = !rst_n;

    wire [31:0] mem_addr;
    wire mem_read;
    wire mem_write;
    wire [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_rdata;

    wire mem_ready;
    
    wire [31:0] serial_rdata;
    wire [31:0] uart_rdata;

    wire flash_cs_n;
    wire ram_a_cs_n;
    wire ram_b_cs_n;

    wire mem_sclk;
    wire mem_mosi;
    
    wire uart_tx;

    wire serial_ready;

    wire flash_select = mem_addr[31:24] == 8'h00;
    wire ram_select = mem_addr[31:24] == 8'h40;
    wire serial_select = flash_select || ram_select;

    reg [31:0] mmio_rdata_reg;
    always @(posedge clk) begin
        mmio_rdata_reg <= uart_rdata;
    end

    assign mem_rdata = serial_select ? serial_rdata : mmio_rdata_reg;
    assign mem_ready = serial_select ? (flash_select && mem_write ? 1'b1 : serial_ready) : 1'b1;

    RiscV #(.WAIT_FOR_MEMORY(1)) cpu (
        .clk(clk),
        .reset(reset),
        .mem_addr(mem_addr),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_wmask(mem_wmask),
        .mem_wdata(mem_wdata),
        .mem_rdata(mem_rdata),
        .mem_ready(mem_ready)
    );

    SerialMemory #(.STARTUP_CYCLES(8192)) memory (
        .clk(clk),
        .reset(reset),
        .valid((mem_read || mem_write) && serial_select && !(flash_select && mem_write)),
        .address(mem_addr),
        .write(mem_write),
        .wdata(mem_wdata),
        .wstrb(mem_wmask),
        .ready(serial_ready),
        .rdata(serial_rdata),
        .flash_cs_n(flash_cs_n),
        .ram_a_cs_n(ram_a_cs_n),
        .ram_b_cs_n(ram_b_cs_n),
        .sclk(mem_sclk),
        .mosi(mem_mosi),
        .miso(uio_in[2])
    );

    Uart uart (
        .clk(clk),
        .reset(reset),
        .address(mem_addr),
        .write_data(mem_wdata),
        .write_mask(mem_wmask),
        .write_enable(mem_write && !serial_select),
        .read_commit(mem_read && !serial_select),
        .rx(ui_in[1]),
        .tx(uart_tx),
        .rx_valid(),
        .read_data(uart_rdata)
    );

    assign uio_out = {ram_b_cs_n, ram_a_cs_n, 2'b00, mem_sclk, 1'b0, mem_mosi, flash_cs_n};
    assign uio_oe = 8'b11001011;
    assign uo_out = {7'b0, uart_tx};
    wire _unused = &{ena, ui_in[7:2], ui_in[0], uio_in[7:3], uio_in[1:0], 1'b0};
endmodule

`default_nettype wire

module RiscVSoc #(
    parameter MEM_WORDS = 2048,
    parameter MEM_INIT_FILE = "firmware.hex",
    parameter GPIO_WIDTH = 8
) (
    input clk,
    input reset,

    input [GPIO_WIDTH-1:0] gpio_in,
    output [GPIO_WIDTH-1:0] gpio_out,
    output [GPIO_WIDTH-1:0] gpio_oe,

    output tx_pin,
    input rx_pin
);
    wire [31:0] mem_addr;
    wire mem_read;
    wire mem_write;
    wire [3:0] mem_wmask;
    wire [31:0] mem_wdata;
    wire [31:0] mem_rdata;
    
    wire [31:0] ram_rdata;
    wire [31:0] gpio_rdata;
    wire [31:0] uart_rdata;

    wire ram_select = mem_addr < MEM_WORDS*4;

    reg [31:0] mmio_rdata_reg;
    always @(posedge clk) begin
        mmio_rdata_reg <= gpio_rdata | uart_rdata;
    end

    assign mem_rdata = ram_select ? ram_rdata : mmio_rdata_reg;

    RiscV core (
        .clk(clk),
        .reset(reset),
        .mem_addr(mem_addr),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_wmask(mem_wmask),
        .mem_wdata(mem_wdata),
        .mem_rdata(mem_rdata),
        .mem_ready(1'b1)
    );

    Sram #(.MEM_WORDS(MEM_WORDS), .MEM_INIT_FILE(MEM_INIT_FILE)) ram (
        .clk(clk),
        .address(mem_addr),
        .read_enable(mem_read && ram_select),
        .read_data(ram_rdata),
        .write_enable(mem_write && ram_select),
        .write_data(mem_wdata),
        .write_mask(mem_wmask)
    );

    Gpio #(.WIDTH(GPIO_WIDTH)) gpio (
        .clk(clk),
        .reset(reset),
        .address(mem_addr),
        .write_data(mem_wdata),
        .write_mask(mem_wmask),
        .write_enable(mem_write && !ram_select),
        .pins_in(gpio_in),
        .pins_in_sync(),
        .pins_out(gpio_out),
        .pins_oe(gpio_oe),
        .read_data(gpio_rdata)
    );

    Uart uart (
        .clk(clk),
        .reset(reset),
        .address(mem_addr),
        .write_data(mem_wdata),
        .write_mask(mem_wmask),
        .write_enable(mem_write && !ram_select),
        .read_commit(mem_read && !ram_select),
        .rx(rx_pin),
        .tx(tx_pin),
        .rx_valid(),
        .read_data(uart_rdata)
    );

endmodule

module Alu (
    input clk,
    input reset,
    input start,
    input [2:0] funct3,
    input subtract,

    input [31:0] a,
    input [31:0] b,

    output reg [31:0] result,
    output reg lt,
    output reg ltu,
    output reg eq,
    output reg done
);
    reg [4:0] bit_idx;
    reg busy;
    reg carry;
    reg zero_so_far;

    wire b_eff = subtract ? ~b[bit_idx] : b[bit_idx];
    wire a_bit = a[bit_idx];
    wire sum_bit = a_bit ^ b_eff ^ carry;
    wire carry_next = (a_bit & b_eff) | (a_bit & carry) | (b_eff & carry);

    reg out_bit;
    always @(*) begin
        case (funct3)
            3'b100:  out_bit = a_bit ^ b[bit_idx];
            3'b110:  out_bit = a_bit | b[bit_idx];
            3'b111:  out_bit = a_bit & b[bit_idx];
            default: out_bit = sum_bit;
        endcase
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            busy <= 0;
            done <= 0;
        end else if (start && !busy) begin
            busy <= 1;
            done <= 0;
            bit_idx <= 0;
            carry <= subtract;
            zero_so_far <= 1'b1;
        end else if (busy) begin
            result <= {out_bit, result[31:1]};
            carry <= carry_next;
            zero_so_far <= zero_so_far & ~sum_bit;
            bit_idx <= bit_idx + 1'b1;

            if (bit_idx == 5'd31) begin
                busy <= 0;
                done <= 1;
                ltu <= ~carry_next;
                lt <= (a[31] ^ b[31]) ? a[31] : ~carry_next;
                eq <= zero_so_far & ~sum_bit;
            end
        end else begin
            done <= 0;
        end
    end
endmodule

module RiscV #(
    parameter WAIT_FOR_MEMORY = 0
) (
    input clk,
    input reset,

    output reg [31:0] mem_addr,
    output reg mem_read,
    output reg mem_write,
    output reg [3:0] mem_wmask,
    output reg [31:0] mem_wdata,
    input [31:0] mem_rdata,
    input mem_ready
);
    localparam S_FETCH      = 4'd0;
    localparam S_FETCH_WAIT = 4'd1;
    localparam S_FETCH2     = 4'd2;
    localparam S_REG1_WAIT  = 4'd3;
    localparam S_REG1       = 4'd4;
    localparam S_REG2_WAIT  = 4'd11;
    localparam S_REG2       = 4'd5;
    localparam S_EXEC_A     = 4'd6;
    localparam S_EXEC_B     = 4'd7;
    localparam S_EXEC_C     = 4'd8;
    localparam S_MEM        = 4'd9;
    localparam S_MEM2       = 4'd10;
    localparam S_WRITEBACK  = 4'd12;
`ifdef CORE_EXTENSION_C
    localparam S_FETCH_CROSS_WAIT = 4'd13;
    localparam S_FETCH_CROSS = 4'd14;
`endif

    localparam PASS_A = 2'd0;
    localparam PASS_B = 2'd1;
    localparam PASS_C = 2'd2;

    reg [3:0] state;
    reg [31:0] PC;
    reg [31:0] Instr;
`ifdef CORE_EXTENSION_C
    reg compressed;
    reg [15:0] first_half;
    wire [15:0] fetched_half = PC[1] ? mem_rdata[31:16] : mem_rdata[15:0];
    wire [31:0] decompressed;

    Decompress decompress (
        .c(fetched_half),
        .instruction(decompressed)
    );
`endif

    wire [6:0] opcode = Instr[6:0];
    wire [2:0] funct3 = Instr[14:12];
    wire funct7b5 = Instr[30];
    wire [4:0] rd = Instr[11:7];
    wire [4:0] shamt = Instr[24:20];

    wire is_rtype   = opcode == 7'b0110011;
    wire is_itype   = opcode == 7'b0010011;
    wire is_load    = opcode == 7'b0000011;
    wire is_store   = opcode == 7'b0100011;
    wire is_branch  = opcode == 7'b1100011;
    wire is_jal     = opcode == 7'b1101111;
    wire is_jalr    = opcode == 7'b1100111;
    wire is_lui     = opcode == 7'b0110111;
    wire is_auipc   = opcode == 7'b0010111;
    wire is_shift   = (is_rtype || is_itype) && (funct3 == 3'b001 || funct3 == 3'b101);
    wire is_mext    = is_rtype && Instr[31:25] == 7'b0000001;
`ifdef CORE_EXTENSION_M
    wire mext_enabled = 1'b1;
`else
    wire mext_enabled = 1'b0;
`endif
    wire is_slt     = (is_rtype || is_itype) && (funct3 == 3'b010 || funct3 == 3'b011);
    wire reg_writes_rd = (is_rtype || is_itype || is_load || is_lui || is_auipc || is_jal || is_jalr) && (!is_mext || mext_enabled) && rd != 0;

    reg [31:0] imm;
    always @(*) begin
        case (opcode)
            7'b0000011, 7'b0010011, 7'b1100111: imm = {{20{Instr[31]}}, Instr[31:20]};
            7'b0100011: imm = {{20{Instr[31]}}, Instr[31:25], Instr[11:7]};
            7'b1100011: imm = {{19{Instr[31]}}, Instr[31], Instr[7], Instr[30:25], Instr[11:8], 1'b0};
            7'b0110111, 7'b0010111: imm = {Instr[31:12], 12'b0};
            7'b1101111: imm = {{11{Instr[31]}}, Instr[31], Instr[19:12], Instr[20], Instr[30:21], 1'b0};
            default: imm = 32'd0;
        endcase
    end

    reg [4:0] rf_addr;
    reg [4:0] rf_a3;
    reg rf_we;
    reg [31:0] rf_wd;
    wire [31:0] rf_rd;

    RegisterFile register_file (
        .clk(clk),
        .addr(rf_addr),
        .a3(rf_a3),
        .we3(rf_we),
        .wd3(rf_wd),
        .rd(rf_rd)
    );

    reg [31:0] rs1_data, rs2_data;

    wire alu_subtract_a = (is_rtype && funct3 == 3'b000 && funct7b5) || is_slt || is_branch;
    wire [31:0] alu_a_operand = is_auipc ? PC : rs1_data;
    wire [31:0] alu_b_operand = (is_rtype || is_branch) ? rs2_data : imm;

    reg [1:0] pass_sel;
    wire [31:0] shared_a = (pass_sel == PASS_A) ? alu_a_operand : PC;
    wire [31:0] shared_b = (pass_sel == PASS_A) ? alu_b_operand :
        (pass_sel == PASS_B) ?
    `ifdef CORE_EXTENSION_C
        (compressed ? 32'd2 : 32'd4) :
    `else
        32'd4 :
    `endif
        imm;
    wire shared_subtract = (pass_sel == PASS_A) ? alu_subtract_a : 1'b0;
    wire [2:0] shared_funct3 = (pass_sel == PASS_A) ? funct3 : 3'b000;

    reg alu_start;
    reg shift_start;
    wire alu_done;
    wire shift_done;
    wire [31:0] alu_result;
    wire [31:0] shift_result;
    wire alu_lt;
    wire alu_ltu;
    wire alu_eq;

    Alu alu (
        .clk(clk),
        .reset(reset),
        .start(alu_start),
        .funct3(shared_funct3),
        .subtract(shared_subtract),
        .a(shared_a),
        .b(shared_b),
        .result(alu_result),
        .lt(alu_lt),
        .ltu(alu_ltu),
        .eq(alu_eq),
        .done(alu_done)
    );

    Shifter shifter (
        .clk(clk),
        .reset(reset),
        .start(shift_start),
        .right(funct3 == 3'b101),
        .arith(funct7b5),
        .operand(rs1_data),
        .shamt(is_itype ? shamt : rs2_data[4:0]),
        .result(shift_result),
        .done(shift_done)
    );

`ifdef CORE_EXTENSION_M
    reg muldiv_start;
    wire muldiv_done;
    wire [31:0] muldiv_result;

    MulDiv muldiv (
        .clk(clk), .reset(reset), .start(muldiv_start),
        .funct3(funct3), .a(rs1_data), .b(rs2_data),
        .result(muldiv_result), .done(muldiv_done)
    );
`endif

    reg [31:0] pass_a_reg;
    reg slt_result_reg;
    reg branch_taken_reg;
    reg [31:0] pc_plus4_reg;
    reg [31:0] pc_plus_imm_reg;

    wire [31:0] pass_a_result = is_shift ? shift_result : pass_a_reg;

    reg branch_taken;
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = alu_eq;
            3'b001: branch_taken = !alu_eq;
            3'b100: branch_taken = alu_lt;
            3'b101: branch_taken = !alu_lt;
            3'b110: branch_taken = alu_ltu;
            3'b111: branch_taken = !alu_ltu;
            default: branch_taken = 1'b0;
        endcase
    end

    wire need_pass_a = (is_rtype || is_itype || is_load || is_store || is_branch || is_jalr || is_auipc) && (!is_mext || mext_enabled);
    wire need_pass_c = is_jal || (is_branch && branch_taken_reg);

    reg [31:0] mem_addr_reg;
    wire [3:0] store_mask;
    wire [31:0] store_data;
    wire [31:0] load_data;

    MaskUnit mask_unit (
        .funct3(funct3),
        .DataAddr(mem_addr_reg),
        .WriteMask(store_mask),
        .ReadData(mem_rdata),
        .MaskedReadData(load_data)
    );
    
    assign store_data = rs2_data << {pass_a_result[1:0], 3'b000};

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= S_FETCH;
            PC <= 32'd0;
            mem_read <= 1'b0;
            mem_write <= 1'b0;
            alu_start <= 1'b0;
            shift_start <= 1'b0;
        `ifdef CORE_EXTENSION_M
            muldiv_start <= 1'b0;
        `endif
            rf_we <= 1'b0;
        end else begin
            alu_start <= 1'b0;
            shift_start <= 1'b0;
        `ifdef CORE_EXTENSION_M
            muldiv_start <= 1'b0;
        `endif
            rf_we <= 1'b0;
            mem_read <= 1'b0;
            mem_write <= 1'b0;

            case (state)
                S_FETCH: begin
                `ifdef CORE_EXTENSION_C
                    mem_addr <= {PC[31:2], 2'b00};
                `else
                    mem_addr <= PC;
                `endif
                    mem_read <= 1'b1;
                    state <= S_FETCH_WAIT;
                end

                S_FETCH_WAIT: begin
                    if (!WAIT_FOR_MEMORY || mem_ready) begin
                        state <= S_FETCH2;
                    end else begin
                        mem_read <= 1'b1;
                    end
                end

                S_FETCH2: begin
                `ifdef CORE_EXTENSION_C
                    if (fetched_half[1:0] != 2'b11) begin
                        Instr <= decompressed;
                        rf_addr <= decompressed[19:15];
                        compressed <= 1'b1;
                        state <= S_REG1_WAIT;
                    end else if (PC[1]) begin
                        first_half <= fetched_half;
                        mem_addr <= {PC[31:2], 2'b00} + 32'd4;
                        mem_read <= 1'b1;
                        state <= S_FETCH_CROSS_WAIT;
                    end else begin
                        Instr <= mem_rdata;
                        rf_addr <= mem_rdata[19:15];
                        compressed <= 1'b0;
                        state <= S_REG1_WAIT;
                    end
                `else
                    Instr <= mem_rdata;
                    rf_addr <= mem_rdata[19:15]; // rs1
                    state <= S_REG1_WAIT;
                `endif
                end

            `ifdef CORE_EXTENSION_C
                S_FETCH_CROSS_WAIT: begin
                    if (!WAIT_FOR_MEMORY || mem_ready) begin
                        state <= S_FETCH_CROSS;
                    end else begin
                        mem_read <= 1'b1;
                    end
                end

                S_FETCH_CROSS: begin
                    Instr <= {mem_rdata[15:0], first_half};
                    rf_addr <= {mem_rdata[3:0], first_half[15]};
                    compressed <= 1'b0;
                    state <= S_REG1_WAIT;
                end
            `endif

                S_REG1_WAIT: begin
                    state <= S_REG1;
                end

                S_REG1: begin
                    rs1_data <= rf_rd;
                    rf_addr <= Instr[24:20]; // rs2
                    state <= S_REG2_WAIT;
                end

                S_REG2_WAIT: begin
                    state <= S_REG2;
                end

                S_REG2: begin
                    rs2_data <= rf_rd;
                    if (need_pass_a) begin
                    `ifdef CORE_EXTENSION_M
                        if (is_mext) begin
                            muldiv_start <= 1'b1;
                        end else
                    `endif
                        if (is_shift) begin
                            shift_start <= 1'b1;
                        end else begin
                            pass_sel <= PASS_A;
                            alu_start <= 1'b1;
                        end
                        state <= S_EXEC_A;
                    end else begin
                        pass_sel <= PASS_B;
                        alu_start <= 1'b1;
                        state <= S_EXEC_B;
                    end
                end

                S_EXEC_A: begin
                `ifdef CORE_EXTENSION_M
                    if (is_mext ? muldiv_done : (is_shift ? shift_done : alu_done)) begin
                        pass_a_reg <= is_mext ? muldiv_result : alu_result;
                `else
                    if (is_shift ? shift_done : alu_done) begin
                        pass_a_reg <= alu_result;
                `endif
                        slt_result_reg <= (funct3 == 3'b010) ? alu_lt : alu_ltu;
                        branch_taken_reg <= branch_taken;

                        pass_sel <= PASS_B;
                        alu_start <= 1'b1;
                        state <= S_EXEC_B;
                    end
                end

                S_EXEC_B: begin
                    if (alu_done) begin
                        pc_plus4_reg <= alu_result;
                        if (need_pass_c) begin
                            pass_sel <= PASS_C;
                            alu_start <= 1'b1;
                            state <= S_EXEC_C;
                        end else begin
                            state <= S_MEM;
                        end
                    end
                end

                S_EXEC_C: begin
                    if (alu_done) begin
                        pc_plus_imm_reg <= alu_result;
                        state <= S_MEM;
                    end
                end

                S_MEM: begin
                    if (is_load) begin
                        mem_addr_reg <= pass_a_result;
                        mem_addr <= pass_a_result;
                        mem_read <= 1'b1;
                        state <= S_MEM2;
                    end else if (is_store) begin
                        mem_addr_reg <= pass_a_result;
                        mem_addr <= pass_a_result;
                        mem_wdata <= store_data;
                        mem_write <= 1'b1;
                        state <= S_WRITEBACK;
                    end else begin
                        state <= S_WRITEBACK;
                    end
                end

                S_MEM2: begin
                    if (!WAIT_FOR_MEMORY || mem_ready) begin
                        state <= S_WRITEBACK;
                    end else begin
                        mem_read <= 1'b1;
                    end
                end

                S_WRITEBACK: begin
                    if (WAIT_FOR_MEMORY && is_store && !mem_ready) begin
                        mem_write <= 1'b1;
                    end else begin
                        if (reg_writes_rd) begin
                            rf_a3 <= rd;
                            rf_we <= 1'b1;
                        `ifdef CORE_EXTENSION_M
                            if (is_mext) begin
                                rf_wd <= pass_a_reg;
                            end else
                        `endif
                            if (is_shift) begin
                                rf_wd <= shift_result;
                            end else if (is_slt) begin
                                rf_wd <= {31'd0, slt_result_reg};
                            end else if (is_load) begin
                                rf_wd <= load_data;
                            end else if (is_lui) begin
                                rf_wd <= imm;
                            end else if (is_jal || is_jalr) begin
                                rf_wd <= pc_plus4_reg;
                            end else begin
                                rf_wd <= pass_a_reg; // r/i-type alu, auipc
                            end
                        end

                        if (is_jalr) begin
                            PC <= {pass_a_reg[31:1], 1'b0};
                        end else if (need_pass_c) begin
                            PC <= pc_plus_imm_reg;
                        end else begin
                            PC <= pc_plus4_reg;
                        end

                        state <= S_FETCH;
                    end
                end

                default: state <= S_FETCH;
            endcase
        end
    end

    always @(*) begin
        mem_wmask = store_mask;
    end
endmodule

`ifdef CORE_EXTENSION_C
module Decompress (
    input [15:0] c,
    output reg [31:0] instruction
);
    localparam [6:0] OP_IMM = 7'h13;
    localparam [6:0] OP = 7'h33;
    localparam [6:0] LUI = 7'h37;
    localparam [6:0] LOAD = 7'h03;
    localparam [6:0] STORE = 7'h23;
    localparam [6:0] BRANCH = 7'h63;
    localparam [6:0] JALR = 7'h67;
    localparam [6:0] JAL = 7'h6f;

    wire [4:0] rd = c[11:7];
    wire [4:0] rs2 = c[6:2];
    wire [4:0] rd_short = {2'b01, c[4:2]};
    wire [4:0] rs1_short = {2'b01, c[9:7]};

    wire [11:0] imm6 = {{7{c[12]}}, c[6:2]};
    wire [11:0] addi4spn_imm = {2'b0, c[10:7], c[12:11], c[5], c[6], 2'b00};
    wire [11:0] word_offset = {5'b0, c[5], c[12:10], c[6], 2'b00};
    wire [11:0] lwsp_offset = {4'b0, c[3:2], c[12], c[6:4], 2'b00};
    wire [11:0] swsp_offset = {4'b0, c[8:7], c[12:9], 2'b00};
    wire [11:0] addi16sp_imm = {{3{c[12]}}, c[4:3], c[5], c[2], c[6], 4'b0};
    wire [20:0] jump_offset = {{10{c[12]}}, c[8], c[10:9], c[6], c[7], c[2], c[11], c[5:3], 1'b0};
    wire [12:0] branch_offset = {{5{c[12]}}, c[6:5], c[2], c[11:10], c[4:3], 1'b0};

    function [31:0] i_type(input [11:0] imm, input [4:0] rs1, input [2:0] funct3, input [4:0] dest, input [6:0] opcode);
        i_type = {imm, rs1, funct3, dest, opcode};
    endfunction

    function [31:0] s_type(input [11:0] imm, input [4:0] src, input [4:0] rs1, input [2:0] funct3, input [6:0] opcode);
        s_type = {imm[11:5], src, rs1, funct3, imm[4:0], opcode};
    endfunction

    function [31:0] r_type(input [6:0] funct7, input [4:0] src, input [4:0] rs1, input [2:0] funct3, input [4:0] dest);
        r_type = {funct7, src, rs1, funct3, dest, OP};
    endfunction

    function [31:0] b_type(input [12:0] imm, input [4:0] rs1, input [2:0] funct3);
        b_type = {imm[12], imm[10:5], 5'd0, rs1, funct3, imm[4:1], imm[11], BRANCH};
    endfunction

    function [31:0] j_type(input [20:0] imm, input [4:0] dest);
        j_type = {imm[20], imm[10:1], imm[11], imm[19:12], dest, JAL};
    endfunction

    always @(*) begin
        instruction = 32'd0;

        case ({c[1:0], c[15:13]})
            // Quadrant 0
            5'b00_000: if (addi4spn_imm != 0) begin
                instruction = i_type(addi4spn_imm, 5'd2, 3'd0, rd_short, OP_IMM);
            end
            5'b00_010: instruction = i_type(word_offset, rs1_short, 3'd2, rd_short, LOAD);
            5'b00_110: instruction = s_type(word_offset, rd_short, rs1_short, 3'd2, STORE);

            // Quadrant 1
            5'b01_000: instruction = i_type(imm6, rd, 3'd0, rd, OP_IMM);
            5'b01_001: instruction = j_type(jump_offset, 5'd1);
            5'b01_010: instruction = i_type(imm6, 5'd0, 3'd0, rd, OP_IMM);
            5'b01_011: begin
                if (rd == 2) begin
                    if (addi16sp_imm != 0) begin
                        instruction = i_type(addi16sp_imm, 5'd2, 3'd0, 5'd2, OP_IMM);
                    end
                end else if (imm6 != 0) begin
                    instruction = {{15{c[12]}}, c[6:2], rd, LUI};
                end
            end
            5'b01_100: begin
                case (c[11:10])
                    2'b00: if (!c[12]) begin
                        instruction = i_type({7'h00, c[6:2]}, rs1_short, 3'd5, rs1_short, OP_IMM);
                    end
                    2'b01: if (!c[12]) begin
                        instruction = i_type({7'h20, c[6:2]}, rs1_short, 3'd5, rs1_short, OP_IMM);
                    end
                    2'b10: instruction = i_type(imm6, rs1_short, 3'd7, rs1_short, OP_IMM);
                    2'b11: begin
                        if (!c[12]) begin
                            case (c[6:5])
                                2'b00: instruction = r_type(7'h20, rd_short, rs1_short, 3'd0, rs1_short);
                                2'b01: instruction = r_type(7'h00, rd_short, rs1_short, 3'd4, rs1_short);
                                2'b10: instruction = r_type(7'h00, rd_short, rs1_short, 3'd6, rs1_short);
                                2'b11: instruction = r_type(7'h00, rd_short, rs1_short, 3'd7, rs1_short);
                            endcase
                        end
                    end
                endcase
            end
            5'b01_101: instruction = j_type(jump_offset, 5'd0);
            5'b01_110: instruction = b_type(branch_offset, rs1_short, 3'd0);
            5'b01_111: instruction = b_type(branch_offset, rs1_short, 3'd1);

            // Quadrant 2
            5'b10_000: if (!c[12]) begin
                instruction = i_type({7'h00, c[6:2]}, rd, 3'd1, rd, OP_IMM);
            end
            5'b10_010: if (rd != 0) begin
                instruction = i_type(lwsp_offset, 5'd2, 3'd2, rd, LOAD);
            end
            5'b10_100: begin
                if (!c[12]) begin
                    if (rs2 == 0) begin
                        if (rd != 0) begin
                            instruction = i_type(12'd0, rd, 3'd0, 5'd0, JALR);
                        end
                    end else begin
                        instruction = r_type(7'h00, rs2, 5'd0, 3'd0, rd);
                    end
                end else begin
                    if (rs2 == 0) begin
                        if (rd == 0) begin
                            instruction = 32'h00100073;
                        end else begin
                            instruction = i_type(12'd0, rd, 3'd0, 5'd1, JALR);
                        end
                    end else begin
                        instruction = r_type(7'h00, rs2, rd, 3'd0, rd);
                    end
                end
            end
            5'b10_110: instruction = s_type(swsp_offset, rs2, 5'd2, 3'd2, STORE);

            default: ;
        endcase
    end
endmodule
`endif

module MaskUnit (
    input [2:0] funct3,
    input [31:0] DataAddr,
    output reg [3:0] WriteMask,

    input [31:0] ReadData,
    output reg [31:0] MaskedReadData
);

    wire sign;
    wire isByte;
    assign isByte = funct3[1:0] == 2'b00;
    wire isHalf;
    assign isHalf = funct3[1:0] == 2'b01;



    wire [15:0] half;
    assign half = DataAddr[1] ? ReadData[31:16] : ReadData[15:0];
    wire [7:0] selected_byte;
    assign selected_byte = DataAddr[0] ? half[15:8] : half[7:0];


    always @(*) begin
        if (isByte) begin
            MaskedReadData = {{24{sign}}, selected_byte};
        end else if (isHalf) begin
            MaskedReadData = {{16{sign}}, half};
        end else begin
            MaskedReadData = ReadData;
        end
    end

    always @(*) begin
        if (isByte) begin
            case (DataAddr[1:0])
                2'b00: WriteMask = 4'b0001;
                2'b01: WriteMask = 4'b0010;
                2'b10: WriteMask = 4'b0100;
                2'b11: WriteMask = 4'b1000;
            endcase
        end else if (isHalf) begin
            if (DataAddr[1]) begin
                WriteMask = 4'b1100;
            end else begin
                WriteMask = 4'b0011;
            end
        end else begin
            WriteMask = 4'b1111;
        end
    end


    assign sign = !funct3[2] & (isByte ? selected_byte[7] : half[15]);

endmodule

`ifdef CORE_EXTENSION_M
module MulDiv (
    input clk,
    input reset,
    input start,
    input [2:0] funct3,
    input [31:0] a,
    input [31:0] b,
    output reg [31:0] result,
    output reg done
);
    localparam IDLE = 4'd0;
    localparam PREP_A = 4'd1;
    localparam PREP_B = 4'd2;
    localparam MUL_ADD = 4'd3;
    localparam MUL_SHIFT = 4'd4;
    localparam DIV_SHIFT = 4'd5;
    localparam DIV_SUB = 4'd6;
    localparam DIV_RESTORE = 4'd7;
    localparam FINISH = 4'd8;
    localparam NEGATE = 4'd9;

    reg [3:0] state;
    reg [4:0] bit_count;
    reg [4:0] step_count;
    reg [2:0] operation;
    reg negate_a;
    reg negate_b;
    reg negate_product;
    reg rem_high;
    reg carry;
    reg [63:0] work;
    reg [31:0] operand;

    wire signed_a = funct3 == 3'b001 || funct3 == 3'b010 || funct3 == 3'b100 || funct3 == 3'b110;
    wire signed_b = funct3 == 3'b001 || funct3 == 3'b100 || funct3 == 3'b110;

    wire [31:0] selected_result = operation[2] ? (operation[1] ? work[63:32] : work[31:0]) : (operation == 3'b000 ? work[31:0] : work[63:32]);
    wire negative_result = operation[2] ? (operation[1] ? negate_a : negate_product) : negate_product;

    reg add_a;
    reg add_b;

    always @(*) begin
        add_a = 1'b0;
        add_b = 1'b0;
    
        case (state)
            PREP_A: add_b = ~work[0];
            PREP_B: add_b = ~operand[0];
    
            MUL_ADD: begin
                add_a = work[32];
                add_b = work[0] & operand[0];
            end
    
            DIV_SUB: begin
                add_a = work[32];
                add_b = ~operand[0];
            end
    
            DIV_RESTORE: begin
                add_a = work[32];
                add_b = operand[0];
            end
    
            NEGATE: add_b = ~result[0];
    
            default: begin end
        endcase
    end
    
    wire sum_bit = add_a ^ add_b ^ carry;
    wire carry_next = (add_a & add_b) | (add_a & carry) | (add_b & carry);
    wire division_negative = rem_high ^ ~carry_next;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: if (start) begin
                    if (funct3[2] && b == 0) begin
                        result <= funct3[1] ? a : 32'hffffffff;
                        done <= 1'b1;
                    end else begin
                        operation <= funct3;
                        negate_a <= signed_a && a[31];
                        negate_b <= signed_b && b[31];
                        negate_product <= (signed_a && a[31]) ^ (signed_b && b[31]);
                        work <= {32'd0, a};
                        operand <= b;
                        bit_count <= 0;
                        step_count <= 0;
                        carry <= 1'b1;
                        state <= PREP_A;
                    end
                end

                PREP_A: begin
                    if (negate_a) begin
                        work[31:0] <= {sum_bit, work[31:1]};
                        carry <= carry_next;
                        bit_count <= bit_count + 1'b1;
                        if (bit_count == 5'd31) begin
                            bit_count <= 0;
                            carry <= 1'b1;
                            state <= PREP_B;
                        end
                    end else begin
                        state <= PREP_B;
                    end
                end

                PREP_B: begin
                    if (negate_b) begin
                        operand <= {sum_bit, operand[31:1]};
                        carry <= carry_next;
                        bit_count <= bit_count + 1'b1;
                        if (bit_count == 5'd31) begin
                            bit_count <= 0;
                            carry <= 1'b0;
                            state <= operation[2] ? DIV_SHIFT : MUL_ADD;
                        end
                    end else begin
                        carry <= 1'b0;
                        state <= operation[2] ? DIV_SHIFT : MUL_ADD;
                    end
                end

                MUL_ADD: begin
                    work[63:32] <= {sum_bit, work[63:33]};
                    operand <= {operand[0], operand[31:1]};
                    carry <= carry_next;
                    bit_count <= bit_count + 1'b1;
                    if (bit_count == 5'd31) begin
                        bit_count <= 0;
                        state <= MUL_SHIFT;
                    end
                end

                MUL_SHIFT: begin
                    work <= {carry, work[63:1]};
                    carry <= 1'b0;
                    step_count <= step_count + 1'b1;
                    state <= step_count == 5'd31 ? FINISH : MUL_ADD;
                end

                DIV_SHIFT: begin
                    rem_high <= work[63];
                    work <= {work[62:0], 1'b0};
                    bit_count <= 0;
                    carry <= 1'b1;
                    state <= DIV_SUB;
                end

                DIV_SUB: begin
                    work[63:32] <= {sum_bit, work[63:33]};
                    operand <= {operand[0], operand[31:1]};
                    carry <= carry_next;
                    bit_count <= bit_count + 1'b1;
                    if (bit_count == 5'd31) begin
                        bit_count <= 0;
                        if (division_negative) begin
                            carry <= 1'b0;
                            state <= DIV_RESTORE;
                        end else begin
                            work[0] <= 1'b1;
                            step_count <= step_count + 1'b1;
                            state <= step_count == 5'd31 ? FINISH : DIV_SHIFT;
                        end
                    end
                end

                DIV_RESTORE: begin
                    work[63:32] <= {sum_bit, work[63:33]};
                    operand <= {operand[0], operand[31:1]};
                    carry <= carry_next;
                    bit_count <= bit_count + 1'b1;
                    if (bit_count == 5'd31) begin
                        bit_count <= 0;
                        step_count <= step_count + 1'b1;
                        state <= step_count == 5'd31 ? FINISH : DIV_SHIFT;
                    end
                end

                FINISH: begin
                    result <= selected_result;
                    if (negative_result) begin
                        carry <= !(!operation[2] && operation != 3'b000 && work[31:0] != 0);
                        bit_count <= 0;
                        state <= NEGATE;
                    end else begin
                        done <= 1'b1;
                        state <= IDLE;
                    end
                end

                NEGATE: begin
                    result <= {sum_bit, result[31:1]};
                    carry <= carry_next;
                    bit_count <= bit_count + 1'b1;
                    if (bit_count == 5'd31) begin
                        done <= 1'b1;
                        state <= IDLE;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule
`endif

module RegisterFile  (
    input clk,

    input [4:0] addr,
    input [4:0] a3,

    input we3,
    input [31:0] wd3,

    output reg [31:0] rd
);
    (* ram_style = "block" *) reg [31:0] rf [31:0];

    wire write = we3 && (a3 != 5'd0);

    always @(posedge clk) begin
        if (write) begin
            rf[a3] <= wd3;
        end
        
        rd <= (addr != 0) ? rf[addr] : 32'd0;
    end
endmodule

module Shifter (
    input clk,
    input reset,
    input start,
    input right,
    input arith,
    input [31:0] operand,
    input [4:0] shamt,

    output reg [31:0] result,
    output reg done
);
    reg [4:0] count;
    reg busy;
    reg signbit;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            busy <= 0;
            done <= 0;
        end else if (start && !busy) begin
            result <= operand;
            signbit <= operand[31];
            count <= shamt;
            busy <= (shamt != 0);
            done <= (shamt == 0);
        end else if (busy) begin
            if (right) begin
                result <= {arith ? signbit : 1'b0, result[31:1]};
            end else begin
                result <= {result[30:0], 1'b0};
            end
            count <= count - 1'b1;
            if (count == 5'd1) begin
                busy <= 0;
                done <= 1;
            end
        end else begin
            done <= 0;
        end
    end
endmodule

module Gpio #(
    parameter BASE_ADDRESS = 32'h80000000,
    parameter WIDTH = 32
) (
    input clk,
    input reset,

    input [31:0] address,
    input [31:0] write_data,
    input [3:0] write_mask,
    input write_enable,

    input [WIDTH-1:0] pins_in,
    output [WIDTH-1:0] pins_in_sync,

    output reg [WIDTH-1:0] pins_out,
    output reg [WIDTH-1:0] pins_oe,

    output reg [31:0] read_data
);
    (* async_reg = "true" *) reg [WIDTH-1:0] pins_meta;
    (* async_reg = "true" *) reg [WIDTH-1:0] pins_sync;

    assign pins_in_sync = pins_sync;

    wire oe_enable = (address[7:0] == BASE_ADDRESS[7:0]);
    wire rw_enable = (address[7:0] == BASE_ADDRESS[7:0] + 8'd4);

    always @* begin
        if (oe_enable) begin
            read_data = {{(32-WIDTH){1'b0}}, pins_oe};
        end else if (rw_enable) begin
            read_data = {{(32-WIDTH){1'b0}}, pins_sync};
        end else begin
            read_data = 0;
        end
    end

    genvar lane;
    generate
        for (lane = 0; lane < WIDTH; lane = lane + 8) begin: byte_lanes
            localparam integer HI = (lane + 7 < WIDTH) ? (lane + 7) : (WIDTH - 1);
            always @(posedge clk or posedge reset) begin
                if (reset) begin
                    pins_oe[HI:lane] <= 0;
                    pins_out[HI:lane] <= 0;
                end else begin
                    if (write_enable && oe_enable && write_mask[lane/8]) begin
                        pins_oe[HI:lane] <= write_data[HI:lane];
                    end

                    if (write_enable && rw_enable && write_mask[lane/8]) begin
                        pins_out[HI:lane] <= write_data[HI:lane];
                    end
                end
            end
        end
    endgenerate

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pins_meta <= 0;
            pins_sync <= 0;
        end else begin
            pins_meta <= pins_in;
            pins_sync <= pins_meta;
        end
    end
endmodule

// Flash: W25Q128JV, 0x00000000-0x00ffffff, read-only.
// PSRAM: two APS6404L chips, 0x40000000-0x40ffffff.
module SerialMemory #(
    parameter STARTUP_CYCLES = 4096
) (
    input wire clk,
    input wire reset,
    input wire valid,
    input wire [31:0] address,
    input wire write,
    input wire [31:0] wdata,
    input wire [3:0] wstrb,
    output wire ready,
    output wire [31:0] rdata,
    output wire flash_cs_n,
    output wire ram_a_cs_n,
    output wire ram_b_cs_n,
    output wire sclk,
    output wire mosi,
    input wire miso
);
    localparam IDLE = 3'd0;
    localparam SETUP = 3'd1;
    localparam SAMPLE = 3'd2;
    localparam GAP = 3'd3;
    localparam DONE = 3'd4;
    localparam INIT_DONE = 3'd4;

    reg [2:0] state;
    reg [13:0] startup_count;
    reg startup_done;
    reg [2:0] init_step;
    reg is_flash;
    reg bank_b;
    reg req_write;
    reg [23:0] word_offset;
    reg [31:0] write_word;
    reg [3:0] remaining_mask;
    reg [1:0] lane;
    reg [39:0] tx_shift;
    reg [31:0] rx_shift;
    reg [6:0] bit_count;

    function [1:0] first_lane;
        input [3:0] mask;
        begin
            if (mask[0]) begin
                first_lane = 0;
            end else if (mask[1]) begin
                first_lane = 1;
            end else if (mask[2]) begin
                first_lane = 2;
            end else begin
                first_lane = 3;
            end
        end
    endfunction

    wire active = state == SETUP || state == SAMPLE;
    assign flash_cs_n = !(active && is_flash);
    assign ram_a_cs_n = !(active && !is_flash && !bank_b);
    assign ram_b_cs_n = !(active && !is_flash && bank_b);

    assign sclk = state == SAMPLE;
    assign mosi = tx_shift[39];
    assign ready = state == DONE;

    assign rdata = {rx_shift[7:0], rx_shift[15:8], rx_shift[23:16], rx_shift[31:24]};

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            startup_count <= 0;
            startup_done <= 0;
            init_step <= 0;
            is_flash <= 0;
            bank_b <= 0;
            req_write <= 0;
            word_offset <= 0;
            write_word <= 0;
            remaining_mask <= 0;
            lane <= 0;
            tx_shift <= 0;
            rx_shift <= 0;
            bit_count <= 0;
        end else begin
            if (!startup_done) begin
                if (startup_count == STARTUP_CYCLES - 1) begin
                    startup_done <= 1;
                end else begin
                    startup_count <= startup_count + 1'b1;
                end
            end

            case (state)
                IDLE: if (startup_done && init_step != INIT_DONE) begin
                    is_flash <= 0;
                    bank_b <= init_step[1];
                    req_write <= 1;
                    remaining_mask <= 0;
                    tx_shift <= {init_step[0] ? 8'h99 : 8'h66, 32'b0};
                    bit_count <= 0;
                    state <= SETUP;
                end else if (startup_done && valid) begin
                    is_flash <= address[31:24] == 0;
                    bank_b <= address[23];
                    req_write <= write;
                    word_offset <= address[23:0];
                    write_word <= wdata;
                    rx_shift <= 0;
                    bit_count <= 0;

                    if (write) begin
                        if (wstrb == 0) begin
                            state <= DONE;
                        end else begin
                            lane <= first_lane(wstrb);
                            remaining_mask <= wstrb & ~(4'b0001 << first_lane(wstrb));
                            tx_shift <= {8'h02, 1'b0, address[22:2], first_lane(wstrb), wdata[first_lane(wstrb)*8 +: 8]};
                            state <= SETUP;
                        end
                    end else begin
                        tx_shift <= {8'h03, address[31:24] == 0 ? {address[23:2], 2'b00} : {1'b0, address[22:2], 2'b00}, 8'h00};
                        state <= SETUP;
                    end
                end

                SETUP: state <= SAMPLE;
                
                SAMPLE: begin
                    if (!req_write && bit_count >= 32) begin
                        rx_shift <= {rx_shift[30:0], miso};
                    end

                    tx_shift <= {tx_shift[38:0], 1'b0};
                    bit_count <= bit_count + 1'b1;
                    
                    if (init_step != INIT_DONE && bit_count == 7) begin
                        init_step <= init_step + 1'b1;
                        state <= IDLE;
                    end else if ((req_write && bit_count == 39) || (!req_write && bit_count == 63)) begin
                        state <= req_write && remaining_mask != 0 ? GAP : DONE;
                    end else begin
                        state <= SETUP;
                    end
                end

                GAP: begin
                    lane <= first_lane(remaining_mask);
                    remaining_mask <= remaining_mask & ~(4'b0001 << first_lane(remaining_mask));
                    tx_shift <= {8'h02, 1'b0, word_offset[22:2], first_lane(remaining_mask), write_word[first_lane(remaining_mask)*8 +: 8]};
                    bit_count <= 0;
                    state <= SETUP;
                end

                DONE: if (!valid) begin
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule

module Sram #(
    parameter MEM_WORDS = 16384,
    parameter MEM_INIT_FILE = "firmware.hex"
) (
    input clk,

    input read_enable,
    input [31:0] address,
    
    output reg [31:0] read_data,
    
    input write_enable,
    input [31:0] write_data,
    input [3:0] write_mask
);
    localparam ADDR_BITS = $clog2(MEM_WORDS);

    (* ram_style = "block" *) reg [31:0] mem [0:MEM_WORDS-1];
    wire in_range = address < MEM_WORDS * 4;
    wire [ADDR_BITS-1:0] word_address = address[ADDR_BITS+1:2];
    
    initial begin
        if (MEM_INIT_FILE != "") begin
            $readmemh(MEM_INIT_FILE, mem);
        end
    end
    
    always @(posedge clk) begin
        if (read_enable) begin
            read_data <= in_range ? mem[word_address] : 32'd0;
        end
        
        if (write_enable && in_range) begin
            if (write_mask[0]) begin
                mem[word_address][7:0] <= write_data[7:0];
            end

            if (write_mask[1]) begin
                mem[word_address][15:8] <= write_data[15:8];
            end

            if (write_mask[2]) begin
                mem[word_address][23:16] <= write_data[23:16];
            end

            if (write_mask[3]) begin
                mem[word_address][31:24] <= write_data[31:24];
            end
        end
    end
endmodule
module Uart #(
    parameter BASE_ADDRESS = 32'h80000008
) (
    input clk,
    input reset,

    input [31:0] address,
    input [31:0] write_data,
    input [3:0] write_mask,
    input write_enable,
    input read_commit,

    input rx,
    output tx,
    output rx_valid,
    output reg [31:0] read_data
);
    wire [7:0] word_address = {address[7:2], 2'b00};
    reg [31:0] scaler;
    wire tx_busy;
    wire tx_start;
    wire rx_clear;
    wire [7:0] rx_data;

    assign tx_start = write_enable && word_address == BASE_ADDRESS[7:0] + 8'd4 && write_mask[0];
    assign rx_clear = read_commit && word_address == BASE_ADDRESS[7:0] + 8'd8;

    always @* begin
        case (word_address)
            BASE_ADDRESS[7:0]: read_data = scaler;
            BASE_ADDRESS[7:0] + 8'd4: read_data = {31'd0, tx_busy};
            BASE_ADDRESS[7:0] + 8'd8: read_data = {16'd0, rx_data, 7'd0, rx_valid};
            default: read_data = 0;
        endcase
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            scaler <= 0;
        end else if (write_enable && word_address == BASE_ADDRESS[7:0]) begin
            if (write_mask[0]) begin
                scaler[7:0] <= write_data[7:0];
            end

            if (write_mask[1]) begin
                scaler[15:8] <= write_data[15:8];
            end

            if (write_mask[2]) begin
                scaler[23:16] <= write_data[23:16];
            end

            if (write_mask[3]) begin
                scaler[31:24] <= write_data[31:24];
            end
        end
    end

    UartTx tx_unit (
        .clk(clk),
        .reset(reset),
        .scaler(scaler[11:0]),
        .start(tx_start),
        .data(write_data[7:0]),
        .tx(tx),
        .busy(tx_busy)
    );

    UartRx rx_unit (
        .clk(clk),
        .reset(reset),
        .scaler(scaler[11:0]),
        .rx(rx),
        .clear(rx_clear),
        .data(rx_data),
        .valid(rx_valid)
    );
endmodule

module UartTx (
    input clk,
    input reset,
    input [11:0] scaler,
    input start,
    input [7:0] data,
    output reg tx,
    output reg busy
);
    localparam IDLE = 2'd0;
    localparam START = 2'd1;
    localparam DATA = 2'd2;
    localparam STOP = 2'd3;

    reg [1:0] state;
    reg [11:0] count;
    reg [11:0] frame_scaler;
    reg [7:0] payload;
    reg [2:0] bitpos;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            tx <= 1'b1;
            busy <= 1'b0;
            count <= 0;
            frame_scaler <= 0;
            payload <= 0;
            bitpos <= 0;
        end else if (state == IDLE) begin
            busy <= 1'b0;
            if (start) begin
                payload <= data;
                frame_scaler <= scaler;
                count <= scaler;
                bitpos <= 0;
                tx <= 1'b0;
                busy <= 1'b1;
                state <= START;
            end
        end else if (count != 0) begin
            count <= count - 1'b1;
        end else begin
            count <= frame_scaler;
            case (state)
                START: begin
                    tx <= payload[0];
                    state <= DATA;
                end
                
                DATA: if (bitpos == 7) begin
                    tx <= 1'b1;
                    state <= STOP;
                end else begin
                    bitpos <= bitpos + 1'b1;
                    tx <= payload[bitpos + 1'b1];
                end

                STOP: state <= IDLE;

                default: begin
                    state <= IDLE;
                    tx <= 1'b1;
                end
            endcase
        end
    end
endmodule

module UartRx (
    input clk,
    input reset,
    input [11:0] scaler,
    input rx,
    input clear,
    output reg [7:0] data,
    output reg valid
);
    localparam IDLE = 2'd0;
    localparam START = 2'd1;
    localparam DATA = 2'd2;
    localparam STOP = 2'd3;

    reg [1:0] state;
    reg [11:0] count;
    reg [2:0] bitpos;
    reg [7:0] shift;
    (* async_reg = "true" *) reg rx_meta;
    (* async_reg = "true" *) reg rx_sync;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            count <= 0;
            bitpos <= 0;
            shift <= 0;
            data <= 0;
            valid <= 0;
            rx_meta <= 1;
            rx_sync <= 1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;

            if (clear) begin
                valid <= 1'b0;
            end

            case (state)
                IDLE: if (!rx_sync) begin
                    count <= (scaler + 1'b1) >> 1;
                    bitpos <= 0;
                    state <= START;
                end

                START: if (count != 0) begin
                    count <= count - 1'b1;
                end else if (!rx_sync) begin
                    count <= scaler;
                    state <= DATA;
                end else begin
                    state <= IDLE;
                end
                
                DATA: if (count != 0) begin
                    count <= count - 1'b1;
                end else begin
                    count <= scaler;
                    shift[bitpos] <= rx_sync;
                    if (bitpos == 7) state <= STOP;
                    else bitpos <= bitpos + 1'b1;
                end
                
                STOP: if (count != 0) begin
                    count <= count - 1'b1;
                end else begin
                    if (rx_sync) begin
                        data <= shift;
                        valid <= 1'b1;
                    end
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule

