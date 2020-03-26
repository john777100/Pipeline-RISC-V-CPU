`timescale 1 ns/10 ps

// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;


//=========================================
	// Note that the overall design of your RISCV includes:
	// 1. pipelined RISCV processor
	// 2. data cache
	// 3. instruction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	

	cache_Improve D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	READ_only_cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule

module RISCV_Pipeline(
		// control interface
		clk            , 
		rst_n          ,
//----------I cache interface-------		
		ICACHE_ren     ,
		ICACHE_wen     ,
		ICACHE_addr    ,
		ICACHE_wdata   ,
		ICACHE_stall   ,
		ICACHE_rdata   ,
//----------D cache interface-------
		DCACHE_ren     ,
		DCACHE_wen     ,
		DCACHE_addr    ,
		DCACHE_wdata   ,
		DCACHE_stall   ,
		DCACHE_rdata   
);

	input			clk, rst_n;
	input	       	ICACHE_stall;
	input	[31:0] 	ICACHE_rdata;
	input	       	DCACHE_stall;
	input	[31:0] 	DCACHE_rdata;

	output        	ICACHE_ren;
	output        	ICACHE_wen;
	output	[29:0] 	ICACHE_addr;
	output 	[31:0] 	ICACHE_wdata;
	output        	DCACHE_ren;
	output        	DCACHE_wen;
	output	[29:0] 	DCACHE_addr;
	output 	[31:0] 	DCACHE_wdata;

	// WIRE DECLARATION 
	//// STALL UNIT
	wire PC_stall, IFID_stall, IDEX_stall, EXMEM_stall, MEMWB_stall;
	wire IFID_flush, IDEX_flush, EXMEM_flush, MEMWB_flush;

	wire	[4:0] 	has_stall;
	wire	[4:0]	has_zero;
	assign	PC_stall 	= has_stall[0];
	assign	IFID_stall 	= has_stall[1];
	assign	IDEX_stall	= has_stall[2];
	assign	EXMEM_stall	= has_stall[3];
	assign	MEMWB_stall	= has_stall[4];
	assign	IFID_flush 	= has_zero[1];
	assign	IDEX_flush	= has_zero[2];
	assign	EXMEM_flush	= has_zero[3];
	assign	MEMWB_flush	= has_zero[4];

	//// HU
	wire			HU_stall;

	//// JumpBranch
	wire	[31:0]	JumpBranch_addr;
	wire			JumpBranch_jump;
	wire			JumpBranch_stall;
	wire			JumpBranch_flush;
	wire			JumpBranch_link;
	wire			JumpBranch_branch;
	wire			JumpBranch_JALR;

	//// FU
	wire			EX_Rs_forw;
	wire	[31:0]	EX_Rs_forw_data;
	wire			EX_Rt_forw;
	wire	[31:0]	EX_Rt_forw_data;
	wire			ID_Rs_forw;
	wire	[31:0]	ID_Rs_forw_data;
	wire			ID_Rt_forw;
	wire	[31:0]	ID_Rt_forw_data;


	//// MAIN CONTROL

    wire	[3:0]   CTRL_ALUctrl;
    wire			CTRL_MemRead;
    wire			CTRL_MemtoReg;
    wire			CTRL_MemWrite;
    wire			CTRL_ALUSrc;
    wire			CTRL_RegWrite;
    wire			CTRL_Rs_valid;
    wire			CTRL_Rt_valid;
    wire			CTRL_Rd_valid;

	//// Reg32

	wire	[31:0]	Reg32_data1;
	wire	[31:0]	Reg32_data2;
	



	//// IF stage
	wire	[31:0] IF_PC, IF_PC4;
	assign	ICACHE_ren 		= 1'b1;
	assign	ICACHE_wen 		= 1'b0;
	assign	ICACHE_addr 	= IF_PC[31:2];
	assign	ICACHE_wdata	= 0;
	

	//// ID Stage
	wire 	[31:0] 	ID_instruction;
	wire	[31:0] 	ID_imm;
	wire	[31:0] 	ID_PC, ID_PC4;
	wire	[31:0] 	ID_data1, ID_data2;
	wire			ID_dataeuq;
	assign			ID_dataeuq = (ID_data1 == ID_data2);

	//// EX stage
	wire			EX_is_bubble;
    wire			EX_ctrl_ALUSrc;
    wire	[3:0]	EX_ctrl_ALUCtrl;
    wire			EX_ctrl_MemWrite;
    wire			EX_ctrl_MemRead;
    wire			EX_ctrl_Mem2Reg;
    wire			EX_ctrl_RegWrite;

    wire	[31:0]	EX_regdata1;
    wire	[31:0]	EX_regdata2;
    wire	[4:0]	EX_Rd;
    wire	[4:0]	EX_Rs;
    wire	[4:0]	EX_Rt;
    wire			EX_Rs_valid;
    wire			EX_Rt_valid;
    wire			EX_JumpBranch_link;
    wire	[31:0]	EX_imm;
    wire	[31:0]	EX_PC4;

	wire	[31:0]	EX_ALU_source1;
	wire	[31:0]	EX_ALU_source2;
	wire	[31:0]	EX_ALU_source21;
	wire	[31:0]	EX_ALU_out;
	wire	[31:0]	EX_ALU_result;


	//// MEM stage
    wire			MEM_ctrl_MemWrite;
    wire			MEM_ctrl_MemRead;
    wire			MEM_ctrl_Mem2Reg;
    wire			MEM_ctrl_RegWrite;
	wire	[31:0]  MEM_data2Mem;
	wire	[4:0]	MEM_Rd;
	wire	[31:0]	MEM_ALU_result;
    wire            MEM_JumpBranch_link;
    wire    [31:0]  MEM_PC4;
    wire    [31:0]  MEM_muxout_data;

	assign	DCACHE_ren 	= MEM_ctrl_MemRead;
	assign 	DCACHE_wen 	= MEM_ctrl_MemWrite;
	assign	DCACHE_addr = MEM_muxout_data[31:2];
	assign	DCACHE_wdata = {MEM_data2Mem[7:0], MEM_data2Mem[15:8], MEM_data2Mem[23:16], MEM_data2Mem[31:24]};


	//// WB stage
    wire			WB_ctrl_Mem2Reg;
    wire			WB_ctrl_RegWrite;
    wire	[31:0]	WB_cache_data;
    wire	[31:0]	WB_ALU_data;
    wire	[4:0]	WB_Rd;
	wire	[31:0]	WB_muxout_data;


	
	//PIPELINE

	PC PC (
		.clk(clk),
		.rst_n(rst_n),
		.PC_jump(JumpBranch_jump),
		.PC_stall(PC_stall),
		.PC_jumpaddr(JumpBranch_addr),
		.PC_addr(IF_PC),
		.PC_PC4(IF_PC4)
	);

	IFID IFID(
		.clk(clk),
		.rst_n(rst_n),
		.IFID_stall(IFID_stall),
		.IFID_flush(IFID_flush),
		.IFID_in_instruction({ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]}),
		.IFID_out_instruction(ID_instruction),
		.IFID_in_PC(IF_PC),
		.IFID_out_PC(ID_PC),
		.IFID_in_PC4(IF_PC4),
		.IFID_out_PC4(ID_PC4)
	);

	IDEX IDEX(
   	 	.clk(clk),
		.rst_n(rst_n),
		.IDEX_stall(IDEX_stall),
		.IDEX_flush(IDEX_flush),
		.IDEX_in_bubble(~ID_instruction[0]),
		.IDEX_out_bubble(EX_is_bubble),
		.IDEX_in_ctrl_ALUSrc(CTRL_ALUSrc),
		.IDEX_out_ctrl_ALUSrc(EX_ctrl_ALUSrc),
		.IDEX_in_ctrl_ALUCtrl(CTRL_ALUctrl),
		.IDEX_out_ctrl_ALUCtrl(EX_ctrl_ALUCtrl),
		.IDEX_in_ctrl_MemWrite(CTRL_MemWrite),
		.IDEX_out_ctrl_MemWrite(EX_ctrl_MemWrite),
		.IDEX_in_ctrl_MemRead(CTRL_MemRead),
		.IDEX_out_ctrl_MemRead(EX_ctrl_MemRead),
		.IDEX_in_ctrl_Mem2Reg(CTRL_MemtoReg),
		.IDEX_out_ctrl_Mem2Reg(EX_ctrl_Mem2Reg),
		.IDEX_in_ctrl_RegWrite(CTRL_RegWrite),
		.IDEX_out_ctrl_RegWrite(EX_ctrl_RegWrite),
		.IDEX_in_regdata1(ID_data1),
		.IDEX_out_regdata1(EX_regdata1),
		.IDEX_in_regdata2(ID_data2),
		.IDEX_out_regdata2(EX_regdata2),
		.IDEX_in_Rd(ID_instruction[11:7]),
		.IDEX_out_Rd(EX_Rd),
		.IDEX_in_Rs(ID_instruction[19:15]),
		.IDEX_out_Rs(EX_Rs),
		.IDEX_in_Rt(ID_instruction[24:20]),
		.IDEX_out_Rt(EX_Rt),
		.IDEX_in_Rs_valid(CTRL_Rs_valid),
		.IDEX_out_Rs_valid(EX_Rs_valid),
		.IDEX_in_Rt_valid(CTRL_Rt_valid),
		.IDEX_out_Rt_valid(EX_Rt_valid),
		.IDEX_in_JumpBranch_link(JumpBranch_link),
		.IDEX_out_JumpBranch_link(EX_JumpBranch_link),
		.IDEX_in_imm(ID_imm),
		.IDEX_out_imm(EX_imm),
		.IDEX_in_PC4(ID_PC4),
		.IDEX_out_PC4(EX_PC4)
	); 

	MainControl MainControl(
		.CTRL_opcode(ID_instruction[6:0]),
		.CTRL_funct7(ID_instruction[31:25]),
		.CTRL_funct3(ID_instruction[14:12]),
		.CTRL_ALUctrl(CTRL_ALUctrl),
		.CTRL_MemRead(CTRL_MemRead),
		.CTRL_MemtoReg(CTRL_MemtoReg),
		.CTRL_MemWrite(CTRL_MemWrite),
		.CTRL_ALUSrc(CTRL_ALUSrc),
		.CTRL_RegWrite(CTRL_RegWrite),
		.CTRL_rs_eff(CTRL_Rs_valid),
		.CTRL_rt_eff(CTRL_Rt_valid)
	);

	Reg32 Reg32 (
		.clk(clk),
		.rst_n(rst_n),
		.RegWrite(WB_ctrl_RegWrite),
		.read_register1(ID_instruction[19:15]),
		.read_register2(ID_instruction[24:20]),
		.write_register(WB_Rd),
		.write_data(WB_muxout_data),
		.read_data1(Reg32_data1),
		.read_data2(Reg32_data2)
	);
	ImmGen ImmGen(
		.ImmGen_instruction(ID_instruction),
		.ImmGen_imm(ID_imm)
	);

	MUX21 ID_MUX_data1(
		.MUX21_ctrl(ID_Rs_forw),
		.MUX21_normaldata(Reg32_data1),
		.MUX21_forwdata(ID_Rs_forw_data),
		.MUX21_out(ID_data1)
	);

	MUX21 ID_MUX_data2(
		.MUX21_ctrl(ID_Rt_forw),
		.MUX21_normaldata(Reg32_data2),
		.MUX21_forwdata(ID_Rt_forw_data),
		.MUX21_out(ID_data2)
	);

	EXMEM EXMEM (
		.clk(clk),
		.rst_n(rst_n),
		.EXMEM_stall(EXMEM_stall),
		.EXMEM_flush(EXMEM_flush),
		.EXMEM_in_ctrl_MemWrite(EX_ctrl_MemWrite),
		.EXMEM_out_ctrl_MemWrite(MEM_ctrl_MemWrite),
		.EXMEM_in_ctrl_MemRead(EX_ctrl_MemRead),
		.EXMEM_out_ctrl_MemRead(MEM_ctrl_MemRead),
		.EXMEM_in_ctrl_Mem2Reg(EX_ctrl_Mem2Reg),
		.EXMEM_out_ctrl_Mem2Reg(MEM_ctrl_Mem2Reg),
		.EXMEM_in_ctrl_RegWrite(EX_ctrl_RegWrite),
		.EXMEM_out_ctrl_RegWrite(MEM_ctrl_RegWrite),
		.EXMEM_in_data2Mem(EX_ALU_source21),
		.EXMEM_out_data2Mem(MEM_data2Mem),
		.EXMEM_in_Rd(EX_Rd),
		.EXMEM_out_Rd(MEM_Rd),
		.EXMEM_in_ALUresult(EX_ALU_result),
		.EXMEM_out_ALUresult(MEM_ALU_result),
        .EXMEM_in_JumpBranch_link(EX_JumpBranch_link),
        .EXMEM_out_JumpBranch_link(MEM_JumpBranch_link),
        .EXMEM_in_PC4(EX_PC4),
        .EXMEM_out_PC4(MEM_PC4)

	);
	ALU ALU(
		.ALU_ctrl(EX_ctrl_ALUCtrl),
		.ALU_data1(EX_ALU_source1),
		.ALU_data2(EX_ALU_source2),
		.ALU_result(EX_ALU_result)
	);
	// MUX21 EX_MUX_source1(
	// 	.MUX21_ctrl(EX_Rs_forw),
	// 	.MUX21_normaldata(EX_regdata1),
	// 	.MUX21_forwdata(EX_Rs_forw_data),
	// 	.MUX21_out(EX_ALU_source1)
	// );
    assign EX_ALU_source1 = EX_Rs_forw ? EX_Rs_forw_data : EX_regdata1;

	// MUX21 EX_MUX_source2(
	// 	.MUX21_ctrl(EX_ctrl_ALUSrc),
	// 	.MUX21_normaldata(EX_ALU_source21),
	// 	.MUX21_forwdata(EX_imm),
	// 	.MUX21_out(EX_ALU_source2)
	// );

    assign EX_ALU_source2 = EX_ctrl_ALUSrc ? EX_imm : EX_ALU_source21;

	// MUX21 EX_MUX_source21(
	// 	.MUX21_ctrl(EX_Rt_forw),
	// 	.MUX21_normaldata(EX_regdata2),
	// 	.MUX21_forwdata(EX_Rt_forw_data),
	// 	.MUX21_out(EX_ALU_source21)
	// );
    assign EX_ALU_source21 = EX_Rt_forw ? EX_Rt_forw_data : EX_regdata2;

	// MUX21 MEM_MUX_out(
	// 	.MUX21_ctrl(MEM_JumpBranch_link),
	// 	.MUX21_normaldata(MEM_ALU_result),
	// 	.MUX21_forwdata(MEM_PC4),
	// 	.MUX21_out(MEM_muxout_data)
	// );
    assign MEM_muxout_data = MEM_JumpBranch_link ? MEM_PC4 : MEM_ALU_result;

	MEMWB MEMWB(
		.clk(clk),
		.rst_n(rst_n),
		.MEMWB_stall(MEMWB_stall),
		.MEMWB_flush(MEMWB_flush),
		.MEMWB_in_ctrl_Mem2Reg(MEM_ctrl_Mem2Reg),
		.MEMWB_out_ctrl_Mem2Reg(WB_ctrl_Mem2Reg),
		.MEMWB_in_ctrl_RegWrite(MEM_ctrl_RegWrite),
		.MEMWB_out_ctrl_RegWrite(WB_ctrl_RegWrite),
		.MEMWB_in_cache_data({DCACHE_rdata[7:0],DCACHE_rdata[15:8],DCACHE_rdata[23:16],DCACHE_rdata[31:24]}),
		.MEMWB_out_cache_data(WB_cache_data),
		.MEMWB_in_ALU_data(MEM_muxout_data),
		.MEMWB_out_ALU_data(WB_ALU_data),
		.MEMWB_in_Rd(MEM_Rd),
		.MEMWB_out_Rd(WB_Rd)
	);

	MUX21 WB_MUX_out(
		.MUX21_ctrl(WB_ctrl_Mem2Reg),
		.MUX21_normaldata(WB_ALU_data),
		.MUX21_forwdata(WB_cache_data),
		.MUX21_out(WB_muxout_data)
	);

	HazardUnit HazardUnit(
		.IDEX_MemRead(EX_ctrl_MemRead),
		.IDEX_rd(EX_Rd),
		.IFID_rs(ID_instruction[19:15]),
        .IFID_rs_eff(CTRL_Rs_valid),
		.IFID_rt(ID_instruction[24:20]),
        .IFID_rt_eff(CTRL_Rt_valid),
		.HU_stall(HU_stall)
	);

	JumpBranch JumpBranch(
		.ID_opcode(ID_instruction[6:0]),
		.ID_funct3(ID_instruction[14:12]),
		.ID_rs(ID_instruction[19:15]),
		.ID_rt(ID_instruction[24:20]),
		.ID_is_equal(ID_dataeuq),
		.EX_rd(EX_Rd),
		.EX_rd_eff(EX_ctrl_RegWrite),
		.ID_Imm(ID_imm),
		.ID_PC(ID_PC),
		.ID_data1(ID_data1),
		.JumpBranch_addr(JumpBranch_addr),
		.JumpBranch_jump(JumpBranch_jump),
		.JumpBranch_stall(JumpBranch_stall),
		.JumpBranch_flush(JumpBranch_flush),
		.JumpBranch_link(JumpBranch_link),
		.JumpBranch_branch(JumpBranch_branch),
		.JumpBranch_JALR(JumpBranch_JALR)   
	);
    
	ForwardUnit EX_ForwardUnit(
		.rs_valid(EX_Rs_valid),
		.rt_valid(EX_Rt_valid),
		.in_rs(EX_Rs),
		.in_rt(EX_Rt),
		.EXMEM_en(MEM_ctrl_RegWrite),
		.EXMEM_rd(MEM_Rd),
		.EXMEM_data(MEM_muxout_data),
		.MEMWB_en(WB_ctrl_RegWrite),
		.MEMWB_rd(WB_Rd),
		.MEMWB_data(WB_muxout_data),
		.rs_forward(EX_Rs_forw),
		.rs_forward_data(EX_Rs_forw_data),
		.rt_forward(EX_Rt_forw),
		.rt_forward_data(EX_Rt_forw_data)
	);

	ForwardUnit ID_ForwardUnit(
		.rs_valid(CTRL_Rs_valid),
		.rt_valid(CTRL_Rt_valid),
		.in_rs(ID_instruction[19:15]),
		.in_rt(ID_instruction[24:20]),
		.EXMEM_en(MEM_ctrl_RegWrite),
		.EXMEM_rd(MEM_Rd),
		.EXMEM_data(MEM_muxout_data),
		.MEMWB_en(WB_ctrl_RegWrite),
		.MEMWB_rd(WB_Rd),
		.MEMWB_data(WB_muxout_data),
		.rs_forward(ID_Rs_forw),
		.rs_forward_data(ID_Rs_forw_data),
		.rt_forward(ID_Rt_forw),
		.rt_forward_data(ID_Rt_forw_data)
	);

	StallUnit_Normal StallUnit(
		.JumpBranch_stall(JumpBranch_stall),
		.HU_stall(HU_stall),
		.JumpBranch_flush(JumpBranch_flush),
		.InsCache_stall(ICACHE_stall),
		.DataCache_stall(DCACHE_stall),
		.IFID_is_forward(ID_Rs_forw|ID_Rt_forw),
		.ID_is_bubble(~ID_instruction[0]),
		.EX_is_bubble(EX_is_bubble),
		.MEM_is_lw(MEM_ctrl_MemRead),
		.has_stall(has_stall),
		.has_zero(has_zero)
	);



endmodule

module IFID(
    clk,
    rst_n,
    IFID_stall,
    IFID_flush,
    IFID_in_instruction,
    IFID_out_instruction,
    IFID_in_PC,
    IFID_out_PC,
    IFID_in_PC4,
    IFID_out_PC4
);
    localparam NOP = 32'd0;
    input   clk, rst_n, IFID_stall, IFID_flush;
    input   [31:0]  IFID_in_instruction;
    input   [31:0]  IFID_in_PC4;
    input   [31:0]  IFID_in_PC;
    output  [31:0]  IFID_out_instruction;
    output  [31:0]  IFID_out_PC4;
    output  [31:0]  IFID_out_PC;

    reg     [31:0]  IFID_out_instruction, n_IFID_out_instruction;
    reg     [31:0]  IFID_out_PC4, n_IFID_out_PC4;
    reg     [31:0]  IFID_out_PC, n_IFID_out_PC;

    always@(*) begin
        n_IFID_out_instruction = IFID_in_instruction;
        n_IFID_out_PC4 = IFID_in_PC4;
        n_IFID_out_PC = IFID_in_PC;
        if(IFID_flush) n_IFID_out_instruction = NOP;
        if(IFID_stall) begin
            n_IFID_out_instruction = IFID_out_instruction;
            n_IFID_out_PC4 = IFID_out_PC4;
            n_IFID_out_PC = IFID_out_PC;

        end
    end
    always@(posedge clk) begin
        if(!rst_n) begin
            IFID_out_instruction <= NOP;
            IFID_out_PC4 <= 32'd0;
            IFID_out_PC <= 32'd0;
        end
        else begin
            IFID_out_instruction <= n_IFID_out_instruction;
            IFID_out_PC4 <= n_IFID_out_PC4;
            IFID_out_PC <= n_IFID_out_PC;
        end
    end
    
endmodule

module IDEX(
    clk,
    rst_n,
    IDEX_stall,
    IDEX_flush,
	IDEX_in_bubble,
	IDEX_out_bubble,
    IDEX_in_ctrl_ALUSrc,
    IDEX_out_ctrl_ALUSrc,
    IDEX_in_ctrl_ALUCtrl,
    IDEX_out_ctrl_ALUCtrl,
    IDEX_in_ctrl_MemWrite,
    IDEX_out_ctrl_MemWrite,
    IDEX_in_ctrl_MemRead,
    IDEX_out_ctrl_MemRead,
    IDEX_in_ctrl_Mem2Reg,
    IDEX_out_ctrl_Mem2Reg,
    IDEX_in_ctrl_RegWrite,
    IDEX_out_ctrl_RegWrite,
    IDEX_in_regdata1,
    IDEX_out_regdata1,
    IDEX_in_regdata2,
    IDEX_out_regdata2,
    IDEX_in_Rd,
    IDEX_out_Rd,
    IDEX_in_Rs,
    IDEX_out_Rs,
    IDEX_in_Rt,
    IDEX_out_Rt,
    IDEX_in_Rs_valid,
    IDEX_out_Rs_valid,
    IDEX_in_Rt_valid,
    IDEX_out_Rt_valid,
    IDEX_in_JumpBranch_link,
    IDEX_out_JumpBranch_link,
    IDEX_in_imm,
    IDEX_out_imm,
    IDEX_in_PC4,
    IDEX_out_PC4
);  
    input clk, rst_n, IDEX_stall, IDEX_flush;

    input           IDEX_in_bubble;
    input           IDEX_in_ctrl_ALUSrc;
    input   [3:0]   IDEX_in_ctrl_ALUCtrl;
    input           IDEX_in_ctrl_MemWrite;
    input           IDEX_in_ctrl_MemRead;
    input           IDEX_in_ctrl_Mem2Reg;
    input           IDEX_in_ctrl_RegWrite;
    input   [31:0]  IDEX_in_regdata1;
    input   [31:0]  IDEX_in_regdata2;
    input   [4:0]   IDEX_in_Rd;
    input   [4:0]   IDEX_in_Rs;
    input   [4:0]   IDEX_in_Rt;
    input           IDEX_in_Rs_valid;
    input           IDEX_in_Rt_valid;
    input           IDEX_in_JumpBranch_link;
    input   [31:0]  IDEX_in_imm;
    input   [31:0]  IDEX_in_PC4;

    output          IDEX_out_bubble;
    output          IDEX_out_ctrl_ALUSrc;
    output  [3:0]   IDEX_out_ctrl_ALUCtrl;
    output          IDEX_out_ctrl_MemWrite;
    output          IDEX_out_ctrl_MemRead;
    output          IDEX_out_ctrl_Mem2Reg;
    output          IDEX_out_ctrl_RegWrite;
    output  [31:0]  IDEX_out_regdata1;
    output  [31:0]  IDEX_out_regdata2;
    output  [4:0]   IDEX_out_Rd;
    output  [4:0]   IDEX_out_Rs;
    output  [4:0]   IDEX_out_Rt;
    output          IDEX_out_Rs_valid;
    output          IDEX_out_Rt_valid;
    output          IDEX_out_JumpBranch_link;
    output  [31:0]  IDEX_out_imm;
    output  [31:0]  IDEX_out_PC4;

    reg             IDEX_out_bubble, n_IDEX_out_bubble;
    reg             IDEX_out_ctrl_ALUSrc, n_IDEX_out_ctrl_ALUSrc;
    reg     [3:0]   IDEX_out_ctrl_ALUCtrl, n_IDEX_out_ctrl_ALUCtrl;
    reg             IDEX_out_ctrl_MemWrite, n_IDEX_out_ctrl_MemWrite;
    reg             IDEX_out_ctrl_MemRead, n_IDEX_out_ctrl_MemRead;
    reg             IDEX_out_ctrl_Mem2Reg, n_IDEX_out_ctrl_Mem2Reg;
    reg             IDEX_out_ctrl_RegWrite, n_IDEX_out_ctrl_RegWrite;
    reg     [31:0]  IDEX_out_regdata1, n_IDEX_out_regdata1;
    reg     [31:0]  IDEX_out_regdata2, n_IDEX_out_regdata2;
    reg     [4:0]   IDEX_out_Rd, n_IDEX_out_Rd;
    reg     [4:0]   IDEX_out_Rs, n_IDEX_out_Rs;
    reg     [4:0]   IDEX_out_Rt, n_IDEX_out_Rt;
    reg             IDEX_out_Rs_valid, n_IDEX_out_Rs_valid;
    reg             IDEX_out_Rt_valid, n_IDEX_out_Rt_valid;
    reg             IDEX_out_JumpBranch_link, n_IDEX_out_JumpBranch_link;
    reg     [31:0]  IDEX_out_imm, n_IDEX_out_imm;
    reg     [31:0]  IDEX_out_PC4, n_IDEX_out_PC4;

    always@(*) begin
        n_IDEX_out_bubble           = IDEX_in_bubble;
        n_IDEX_out_ctrl_ALUSrc      = IDEX_in_ctrl_ALUSrc;
        n_IDEX_out_ctrl_ALUCtrl     = IDEX_in_ctrl_ALUCtrl;
        n_IDEX_out_ctrl_MemWrite    = IDEX_in_ctrl_MemWrite;
        n_IDEX_out_ctrl_MemRead     = IDEX_in_ctrl_MemRead;
        n_IDEX_out_ctrl_Mem2Reg     = IDEX_in_ctrl_Mem2Reg;
        n_IDEX_out_ctrl_RegWrite    = IDEX_in_ctrl_RegWrite;
        n_IDEX_out_regdata1         = IDEX_in_regdata1;
        n_IDEX_out_regdata2         = IDEX_in_regdata2;
        n_IDEX_out_Rd               = IDEX_in_Rd;
        n_IDEX_out_Rs               = IDEX_in_Rs;
        n_IDEX_out_Rt               = IDEX_in_Rt;
        n_IDEX_out_Rs_valid         = IDEX_in_Rs_valid;
        n_IDEX_out_Rt_valid         = IDEX_in_Rt_valid;
        n_IDEX_out_JumpBranch_link  = IDEX_in_JumpBranch_link;
        n_IDEX_out_imm              = IDEX_in_imm;
        n_IDEX_out_PC4              = IDEX_in_PC4;

        if(IDEX_flush | IDEX_in_bubble) begin
            n_IDEX_out_bubble           = 1;
            n_IDEX_out_ctrl_ALUSrc      = 0;
            n_IDEX_out_ctrl_ALUCtrl     = 4'b1111;
            n_IDEX_out_ctrl_MemWrite    = 0;
            n_IDEX_out_ctrl_MemRead     = 0;
            n_IDEX_out_ctrl_Mem2Reg     = 0;
            n_IDEX_out_ctrl_RegWrite    = 0;
            n_IDEX_out_regdata1         = IDEX_out_regdata1;
            n_IDEX_out_regdata2         = IDEX_out_regdata2;
            n_IDEX_out_Rd               = IDEX_out_Rd;
            n_IDEX_out_Rs               = IDEX_out_Rs;
            n_IDEX_out_Rt               = IDEX_out_Rt;
            n_IDEX_out_Rs_valid         = 0;
            n_IDEX_out_Rt_valid         = 0;
            n_IDEX_out_JumpBranch_link  = IDEX_out_JumpBranch_link;
            n_IDEX_out_imm              = IDEX_out_imm;
            n_IDEX_out_PC4              = IDEX_out_PC4;
        end
        if(IDEX_stall) begin
            n_IDEX_out_bubble           = IDEX_out_bubble;
            n_IDEX_out_ctrl_ALUSrc      = IDEX_out_ctrl_ALUSrc;
            n_IDEX_out_ctrl_ALUCtrl     = IDEX_out_ctrl_ALUCtrl;
            n_IDEX_out_ctrl_MemWrite    = IDEX_out_ctrl_MemWrite;
            n_IDEX_out_ctrl_MemRead     = IDEX_out_ctrl_MemRead;
            n_IDEX_out_ctrl_Mem2Reg     = IDEX_out_ctrl_Mem2Reg;
            n_IDEX_out_ctrl_RegWrite    = IDEX_out_ctrl_RegWrite;
            n_IDEX_out_regdata1         = IDEX_out_regdata1;
            n_IDEX_out_regdata2         = IDEX_out_regdata2;
            n_IDEX_out_Rd               = IDEX_out_Rd;
            n_IDEX_out_Rs               = IDEX_out_Rs;
            n_IDEX_out_Rs_valid         = IDEX_out_Rs_valid;
            n_IDEX_out_Rt_valid         = IDEX_out_Rt_valid;
            n_IDEX_out_Rt               = IDEX_out_Rt;
            n_IDEX_out_JumpBranch_link  = IDEX_out_JumpBranch_link;
            n_IDEX_out_imm              = IDEX_out_imm;
            n_IDEX_out_PC4              = IDEX_out_PC4;
        end
    end
    always@(posedge clk) begin
        if(!rst_n) begin
            IDEX_out_bubble           <= 1;
            IDEX_out_ctrl_ALUSrc      <= 0;
            IDEX_out_ctrl_ALUCtrl     <= 4'b1111;
            IDEX_out_ctrl_MemWrite    <= 0;
            IDEX_out_ctrl_MemRead     <= 0;
            IDEX_out_ctrl_Mem2Reg     <= 0;
            IDEX_out_ctrl_RegWrite    <= 0;
            IDEX_out_regdata1         <= 0;
            IDEX_out_regdata2         <= 0;
            IDEX_out_Rd               <= 0;
            IDEX_out_Rs               <= 0;
            IDEX_out_Rs_valid         <= 0;
            IDEX_out_Rt_valid         <= 0;
            IDEX_out_Rt               <= 0;
            IDEX_out_JumpBranch_link  <= 0;
            IDEX_out_imm              <= 0;
            IDEX_out_PC4              <= 0;
        end
        else begin
            IDEX_out_bubble           <= n_IDEX_out_bubble;
            IDEX_out_ctrl_ALUSrc      <= n_IDEX_out_ctrl_ALUSrc;
            IDEX_out_ctrl_ALUCtrl     <= n_IDEX_out_ctrl_ALUCtrl;
            IDEX_out_ctrl_MemWrite    <= n_IDEX_out_ctrl_MemWrite;
            IDEX_out_ctrl_MemRead     <= n_IDEX_out_ctrl_MemRead;
            IDEX_out_ctrl_Mem2Reg     <= n_IDEX_out_ctrl_Mem2Reg;
            IDEX_out_ctrl_RegWrite    <= n_IDEX_out_ctrl_RegWrite;
            IDEX_out_regdata1         <= n_IDEX_out_regdata1;
            IDEX_out_regdata2         <= n_IDEX_out_regdata2;
            IDEX_out_Rd               <= n_IDEX_out_Rd;
            IDEX_out_Rs               <= n_IDEX_out_Rs;
            IDEX_out_Rs_valid         <= n_IDEX_out_Rs_valid;
            IDEX_out_Rt_valid         <= n_IDEX_out_Rt_valid;
            IDEX_out_Rt               <= n_IDEX_out_Rt;
            IDEX_out_JumpBranch_link  <= n_IDEX_out_JumpBranch_link;
            IDEX_out_imm              <= n_IDEX_out_imm;
            IDEX_out_PC4              <= n_IDEX_out_PC4;

        end
    end


endmodule

module EXMEM(
    clk,
    rst_n,
    EXMEM_stall,
    EXMEM_flush,
    EXMEM_in_ctrl_MemWrite,
    EXMEM_out_ctrl_MemWrite,
    EXMEM_in_ctrl_MemRead,
    EXMEM_out_ctrl_MemRead,
    EXMEM_in_ctrl_Mem2Reg,
    EXMEM_out_ctrl_Mem2Reg,
    EXMEM_in_ctrl_RegWrite,
    EXMEM_out_ctrl_RegWrite,
    EXMEM_in_data2Mem,
    EXMEM_out_data2Mem,
    EXMEM_in_Rd,
    EXMEM_out_Rd,
    EXMEM_in_ALUresult,
    EXMEM_out_ALUresult,
    EXMEM_in_JumpBranch_link,
    EXMEM_out_JumpBranch_link,
    EXMEM_in_PC4,
    EXMEM_out_PC4
);

    input clk, rst_n, EXMEM_stall, EXMEM_flush;

    input           EXMEM_in_ctrl_MemWrite;
    input           EXMEM_in_ctrl_MemRead;
    input           EXMEM_in_ctrl_Mem2Reg;
    input           EXMEM_in_ctrl_RegWrite;
    input   [31:0]  EXMEM_in_data2Mem;
    input   [4:0]   EXMEM_in_Rd;
    input   [31:0]  EXMEM_in_ALUresult;
    input           EXMEM_in_JumpBranch_link;
    input   [31:0]  EXMEM_in_PC4;

    output          EXMEM_out_ctrl_MemWrite;
    output          EXMEM_out_ctrl_MemRead;
    output          EXMEM_out_ctrl_Mem2Reg;
    output          EXMEM_out_ctrl_RegWrite;
    output  [31:0]  EXMEM_out_data2Mem;
    output  [4:0]   EXMEM_out_Rd;
    output  [31:0]  EXMEM_out_ALUresult;
    output          EXMEM_out_JumpBranch_link;
    output  [31:0]  EXMEM_out_PC4;


    reg             EXMEM_out_ctrl_MemWrite, n_EXMEM_out_ctrl_MemWrite;
    reg             EXMEM_out_ctrl_MemRead, n_EXMEM_out_ctrl_MemRead;
    reg             EXMEM_out_ctrl_Mem2Reg, n_EXMEM_out_ctrl_Mem2Reg;
    reg             EXMEM_out_ctrl_RegWrite, n_EXMEM_out_ctrl_RegWrite;
    reg     [31:0]  EXMEM_out_data2Mem, n_EXMEM_out_data2Mem;
    reg     [4:0]   EXMEM_out_Rd, n_EXMEM_out_Rd;
    reg     [31:0]  EXMEM_out_ALUresult, n_EXMEM_out_ALUresult;
    reg             EXMEM_out_JumpBranch_link, n_EXMEM_out_JumpBranch_link;
    reg     [31:0]  EXMEM_out_PC4, n_EXMEM_out_PC4;



    always@(*) begin
        n_EXMEM_out_ctrl_MemWrite = EXMEM_in_ctrl_MemWrite;
        n_EXMEM_out_ctrl_MemRead = EXMEM_in_ctrl_MemRead;
        n_EXMEM_out_ctrl_Mem2Reg = EXMEM_in_ctrl_Mem2Reg;
        n_EXMEM_out_ctrl_RegWrite = EXMEM_in_ctrl_RegWrite;
        n_EXMEM_out_data2Mem = EXMEM_in_data2Mem;
        n_EXMEM_out_Rd = EXMEM_in_Rd;
        n_EXMEM_out_ALUresult = EXMEM_in_ALUresult;
        n_EXMEM_out_JumpBranch_link = EXMEM_in_JumpBranch_link;
        n_EXMEM_out_PC4 = EXMEM_in_PC4;

        if(EXMEM_flush) begin
            n_EXMEM_out_ctrl_MemWrite = 0;
            n_EXMEM_out_ctrl_MemRead = 0;
            n_EXMEM_out_ctrl_Mem2Reg = 0;
            n_EXMEM_out_ctrl_RegWrite = 0;
            n_EXMEM_out_data2Mem = EXMEM_in_data2Mem;
            n_EXMEM_out_Rd = EXMEM_in_Rd;
            n_EXMEM_out_ALUresult = EXMEM_in_ALUresult;
            n_EXMEM_out_JumpBranch_link = EXMEM_out_JumpBranch_link;
            n_EXMEM_out_PC4 = EXMEM_out_PC4;
        end
        if(EXMEM_stall) begin
            n_EXMEM_out_ctrl_MemWrite = EXMEM_out_ctrl_MemWrite;
            n_EXMEM_out_ctrl_MemRead = EXMEM_out_ctrl_MemRead;
            n_EXMEM_out_ctrl_Mem2Reg = EXMEM_out_ctrl_Mem2Reg;
            n_EXMEM_out_ctrl_RegWrite = EXMEM_out_ctrl_RegWrite;
            n_EXMEM_out_data2Mem = EXMEM_out_data2Mem;
            n_EXMEM_out_Rd = EXMEM_out_Rd;
            n_EXMEM_out_ALUresult = EXMEM_out_ALUresult;
            n_EXMEM_out_JumpBranch_link = EXMEM_out_JumpBranch_link;
            n_EXMEM_out_PC4 = EXMEM_out_PC4;
        end

    end

    always@(posedge clk) begin
        if(!rst_n) begin
            EXMEM_out_ctrl_MemWrite <= 0;
            EXMEM_out_ctrl_MemRead <= 0;
            EXMEM_out_ctrl_Mem2Reg <= 0;
            EXMEM_out_ctrl_RegWrite <= 0;
            EXMEM_out_data2Mem <= 0;
            EXMEM_out_Rd <= 0;
            EXMEM_out_ALUresult <= 0;
            EXMEM_out_JumpBranch_link <= 0;
            EXMEM_out_PC4 <= 0;
        end
        else begin
            EXMEM_out_ctrl_MemWrite <= n_EXMEM_out_ctrl_MemWrite;
            EXMEM_out_ctrl_MemRead <= n_EXMEM_out_ctrl_MemRead;
            EXMEM_out_ctrl_Mem2Reg <= n_EXMEM_out_ctrl_Mem2Reg;
            EXMEM_out_ctrl_RegWrite <= n_EXMEM_out_ctrl_RegWrite;
            EXMEM_out_data2Mem <= n_EXMEM_out_data2Mem;
            EXMEM_out_Rd <= n_EXMEM_out_Rd;
            EXMEM_out_ALUresult <= n_EXMEM_out_ALUresult;
            EXMEM_out_JumpBranch_link <= n_EXMEM_out_JumpBranch_link;
            EXMEM_out_PC4 <= n_EXMEM_out_PC4;

        end
    end

    
endmodule

module MEMWB(
    clk,
    rst_n,
    MEMWB_stall,
    MEMWB_flush,
    MEMWB_in_ctrl_Mem2Reg,
    MEMWB_out_ctrl_Mem2Reg,
    MEMWB_in_ctrl_RegWrite,
    MEMWB_out_ctrl_RegWrite,
    MEMWB_in_cache_data,
    MEMWB_out_cache_data,
    MEMWB_in_ALU_data,
    MEMWB_out_ALU_data,
    MEMWB_in_Rd,
    MEMWB_out_Rd,
);
    input   clk, rst_n, MEMWB_stall, MEMWB_flush;

    input           MEMWB_in_ctrl_Mem2Reg;
    input           MEMWB_in_ctrl_RegWrite;
    input   [31:0]  MEMWB_in_cache_data;
    input   [31:0]  MEMWB_in_ALU_data;
    input   [4:0]   MEMWB_in_Rd;


    output          MEMWB_out_ctrl_Mem2Reg;
    output          MEMWB_out_ctrl_RegWrite;
    output  [31:0]  MEMWB_out_cache_data;
    output  [31:0]  MEMWB_out_ALU_data;
    output  [4:0]   MEMWB_out_Rd;

    reg             MEMWB_out_ctrl_Mem2Reg, n_MEMWB_out_ctrl_Mem2Reg;
    reg             MEMWB_out_ctrl_RegWrite, n_MEMWB_out_ctrl_RegWrite;
    reg     [31:0]  MEMWB_out_cache_data, n_MEMWB_out_cache_data;
    reg     [31:0]  MEMWB_out_ALU_data, n_MEMWB_out_ALU_data;
    reg     [4:0]   MEMWB_out_Rd, n_MEMWB_out_Rd;

    always@(*) begin
        n_MEMWB_out_ctrl_Mem2Reg = MEMWB_in_ctrl_Mem2Reg;
        n_MEMWB_out_ctrl_RegWrite = MEMWB_in_ctrl_RegWrite;
        n_MEMWB_out_cache_data =  MEMWB_in_cache_data;
        n_MEMWB_out_ALU_data =  MEMWB_in_ALU_data;
        n_MEMWB_out_Rd = MEMWB_in_Rd;
        if(MEMWB_flush) begin
            n_MEMWB_out_ctrl_Mem2Reg = 0;
            n_MEMWB_out_ctrl_RegWrite = 0;
            n_MEMWB_out_cache_data =  MEMWB_in_cache_data;
            n_MEMWB_out_ALU_data =  MEMWB_in_ALU_data;
            n_MEMWB_out_Rd = MEMWB_in_Rd;
        end
        if(MEMWB_stall) begin
            n_MEMWB_out_ctrl_Mem2Reg = MEMWB_out_ctrl_Mem2Reg;
            n_MEMWB_out_ctrl_RegWrite = MEMWB_out_ctrl_RegWrite;
            n_MEMWB_out_cache_data =  MEMWB_out_cache_data;
            n_MEMWB_out_ALU_data =  MEMWB_out_ALU_data;
            n_MEMWB_out_Rd = MEMWB_out_Rd;
        end
    
    end
    always@(posedge clk) begin
        if(!rst_n) begin
            MEMWB_out_ctrl_Mem2Reg <= 0;
            MEMWB_out_ctrl_RegWrite <= 0;
            MEMWB_out_cache_data <= 0;
            MEMWB_out_ALU_data <= 0;
            MEMWB_out_Rd <= 0;
        end
        else begin
            MEMWB_out_ctrl_Mem2Reg <= n_MEMWB_out_ctrl_Mem2Reg;
            MEMWB_out_ctrl_RegWrite <= n_MEMWB_out_ctrl_RegWrite;
            MEMWB_out_cache_data <= n_MEMWB_out_cache_data;
            MEMWB_out_ALU_data <= n_MEMWB_out_ALU_data;
            MEMWB_out_Rd <= n_MEMWB_out_Rd;
        end

    end
    
endmodule

module Reg32(
	clk,
	rst_n,
	RegWrite,
	read_register1,
	read_register2,
	write_register,
	write_data,
	read_data1,
	read_data2
);
    input   clk, rst_n, RegWrite;
    input   [4:0] read_register1;
    input   [4:0] read_register2;
    input   [4:0] write_register;
    input   [31:0] write_data;
    output  [31:0] read_data1;
    output  [31:0] read_data2;

    // reg     [31:0] read_data1;
    // reg     [31:0] read_data2;
    reg     [31:0] register [0:31];
    reg     [31:0] next_register [0:31];
    integer i;

    assign read_data1 = register[read_register1];
    assign read_data2 = register[read_register2]; 
    always@(*) begin
        for(i = 1; i <= 31; i = i + 1) begin
            next_register[i] = register[i];
        end
        if(RegWrite) next_register[write_register] = write_data;
        next_register[0] = 0;

    end

    always@(posedge clk) begin
        if(rst_n == 0) begin
            for(i = 0; i <= 31; i = i + 1) begin
                register[i] <= 0;
            end
        end
        else begin
            for(i = 0; i <= 31; i = i + 1) begin
                register[i] <= next_register[i];
            end
        end
    end
endmodule

module ImmGen(
    ImmGen_instruction,
    ImmGen_imm
);
    input   [31:0]  ImmGen_instruction;
    output  [31:0]  ImmGen_imm;
    reg     [31:0]  ImmGen_imm;

    always@(*) begin
        case(ImmGen_instruction[6:0])
            7'b0010011: begin // ADDI, SLTI, XORI, ORI, ANDI, SLLI, SRLI, SRAI, 
                case(ImmGen_instruction[14:12])
                    3'b001: ImmGen_imm = {26'd0, ImmGen_instruction[24:20]}; // SLLI
                    3'b101: ImmGen_imm = {26'd0, ImmGen_instruction[24:20]}; // SRLI, SRAI
                    default:ImmGen_imm = {{{20{ImmGen_instruction[31]}}},ImmGen_instruction[31:20]};
                endcase
            end
            7'b0100011: begin // SW
                ImmGen_imm = {{20{ImmGen_instruction[31]}}, ImmGen_instruction[31:25], ImmGen_instruction[11:7]};
            end
            7'b0000011: begin // LW
                ImmGen_imm = {{20{ImmGen_instruction[31]}}, ImmGen_instruction[31:20]};
            end
            7'b1100011: begin // B-type
                ImmGen_imm = {{20{ImmGen_instruction[31]}}, ImmGen_instruction[7], ImmGen_instruction[30:25], ImmGen_instruction[11:8], 1'b0};
            end
            7'b1100111: begin // JALR
                ImmGen_imm = {{20{ImmGen_instruction[31]}}, ImmGen_instruction[31:20]};
            end
            7'b1101111: begin // JAL
                ImmGen_imm = {{12{ImmGen_instruction[31]}}, ImmGen_instruction[19:12], ImmGen_instruction[20], ImmGen_instruction[30:21], 1'b0 };
            end
            default: ImmGen_imm = 32'd0;
        endcase
    end
endmodule

module MUX21(
    MUX21_ctrl,
    MUX21_normaldata,
    MUX21_forwdata,
    MUX21_out
);
    input   MUX21_ctrl;
    input   [31:0]  MUX21_normaldata;
    input   [31:0]  MUX21_forwdata;
    output  [31:0]  MUX21_out;
    assign MUX21_out = MUX21_ctrl ? MUX21_forwdata : MUX21_normaldata;
    
endmodule

module ALU(
    ALU_ctrl,
    ALU_data1,
    ALU_data2,
    ALU_result
);
    input   [3:0]   ALU_ctrl;
    input   [31:0]  ALU_data1;
    input   [31:0]  ALU_data2;
    output  [31:0]   ALU_result;
    reg     [31:0]   ALU_result;
    reg signed    [31:0] ALU_data1_signed;
    reg signed    [31:0] ALU_data2_signed;


    localparam AND  = 4'b0000;
    localparam OR   = 4'b0001;
    localparam ADD  = 4'b0010;
    localparam SUB  = 4'b0110;
    localparam SOLT = 4'b0111;
    localparam SRA  = 4'b1001;
    localparam SLL  = 4'b1010;
    localparam SRL  = 4'b1011;
    localparam XOR  = 4'b0011;
    localparam NOP  = 4'b1111;


    always@(*) begin
        ALU_data1_signed = ALU_data1;
        ALU_data2_signed = ALU_data2;
        case (ALU_ctrl)
            AND:    ALU_result = ALU_data1_signed &     ALU_data2_signed;
            OR:     ALU_result = ALU_data1_signed |     ALU_data2_signed;
            ADD:    ALU_result = ALU_data1_signed +     ALU_data2_signed;
            SUB:    ALU_result = ALU_data1_signed -     ALU_data2_signed;
            SRA:    ALU_result = ALU_data1_signed >>>   ALU_data2_signed[4:0];
            SLL:    ALU_result = ALU_data1_signed <<    ALU_data2_signed[4:0];
            SRL:    ALU_result = ALU_data1_signed >>    ALU_data2_signed[4:0];
            SOLT:   ALU_result = ALU_data1_signed <     ALU_data2_signed ? 1 : 0;
            XOR:    ALU_result = ALU_data1_signed ^     ALU_data2_signed;
            //NOP:    ALU_result = 0;   
            default:ALU_result = 0; 
        endcase
    end
endmodule

module PC (
    clk,
    rst_n,
    PC_jump,
    PC_stall,
    PC_jumpaddr,
    PC_addr,
    PC_PC4
);
    input   clk, rst_n, PC_jump, PC_stall;
    input   [31:0] PC_jumpaddr;
    output  [31:0] PC_addr, PC_PC4;
    reg     [31:0] PC_addr, n_PC_addr;

    assign PC_PC4 = PC_addr+4;

    always@(*) begin
        n_PC_addr = PC_addr + 4;
        if(PC_jump)  n_PC_addr = PC_jumpaddr;
        if(PC_stall) n_PC_addr = PC_addr;
    end

    always@(posedge clk) begin
       if(!rst_n)   PC_addr <= 32'b0;
       else         PC_addr <= n_PC_addr;
    end
endmodule


/// Chen

module MainControl(
    CTRL_opcode,
    CTRL_funct7,
    CTRL_funct3,
    CTRL_ALUctrl,
    CTRL_MemRead,
    CTRL_MemtoReg,
    CTRL_MemWrite,
    CTRL_ALUSrc,
    CTRL_RegWrite,
    CTRL_rs_eff,
    CTRL_rt_eff
);
    localparam AND  = 4'b0000;
    localparam OR   = 4'b0001;
    localparam ADD  = 4'b0010;
    localparam SUB  = 4'b0110;
    localparam SOLT = 4'b0111;
    localparam SRA  = 4'b1001;
    localparam SLL  = 4'b1010;
    localparam SRL  = 4'b1011;
    localparam XOR  = 4'b0011;
    localparam NOP  = 4'b1111;


    //======I/O Declaration==================

    input [6:0] CTRL_opcode;
    input [6:0] CTRL_funct7;
    input [2:0] CTRL_funct3;
    output [3:0] CTRL_ALUctrl;
    output CTRL_MemRead;
    output CTRL_MemtoReg;
    output CTRL_MemWrite;
    output CTRL_ALUSrc;
    output CTRL_RegWrite;
    output CTRL_rs_eff;
    output CTRL_rt_eff;

    reg    [3:0] CTRL_ALUctrl;
    //======Combinational Part==============

    always@(*) begin
        case ({CTRL_opcode[6], CTRL_opcode[4:0]})
            6'b010011: begin // except B type, J type, S type, lw
                case(CTRL_funct3)
                    3'b000: CTRL_ALUctrl = CTRL_funct7 == 7'b0100000 ? SUB : ADD;
                    3'b001: CTRL_ALUctrl = SLL;
                    3'b101: CTRL_ALUctrl = CTRL_funct7[5] ? SRA : SRL;
                    3'b010: CTRL_ALUctrl = SOLT;
                    3'b100: CTRL_ALUctrl = XOR;
                    3'b110: CTRL_ALUctrl = OR;
                    3'b111: CTRL_ALUctrl = AND;
                    default:CTRL_ALUctrl = NOP;
                endcase
            end            
            default: CTRL_ALUctrl = ADD;
        endcase
    end

    // assign CTRL_ALUctrl[0] = (CTRL_opcode[5] & ~CTRL_funct3[0] & (CTRL_funct3[1] | CTRL_funct3[2])) | 
    //                         (CTRL_funct3[2] & ~CTRL_funct3[1]) | CTRL_opcode[6];
    // assign CTRL_ALUctrl[1] = ((~CTRL_funct3[2] | ~CTRL_funct3[1]) & (~(CTRL_funct3==3'b101) | ~CTRL_funct7[5])) | CTRL_opcode[6];
    // assign CTRL_ALUctrl[2] = (CTRL_opcode[4] & ~CTRL_funct3[2] & CTRL_funct3[1]) | 
    //                         (CTRL_opcode[5] & CTRL_funct7[5] & ~CTRL_funct3[1]) | CTRL_opcode[6];
    // assign CTRL_ALUctrl[3] = (~CTRL_funct3[1] & CTRL_funct3[0]) | CTRL_opcode[6];
    assign CTRL_MemRead = ~CTRL_opcode[5] & ~CTRL_opcode[4];
    assign CTRL_MemtoReg = CTRL_MemRead;
    assign CTRL_MemWrite = ~CTRL_opcode[6] & CTRL_opcode[5] & ~CTRL_opcode[4];
    assign CTRL_ALUSrc = ~(CTRL_opcode[5] & CTRL_opcode[4]);
    assign CTRL_RegWrite = ~(CTRL_opcode[5] & ~CTRL_opcode[4] & ~CTRL_opcode[2]);
    assign CTRL_rs_eff = ~CTRL_opcode[3];
    assign CTRL_rt_eff = CTRL_opcode[5] & ~CTRL_opcode[2];

endmodule

module HazardUnit(
    IDEX_MemRead,
    IDEX_rd,
    IFID_rs,
    IFID_rs_eff,
    IFID_rt,
    IFID_rt_eff,
    HU_stall
);

//======I/O Declaration================

    input IDEX_MemRead;
    input [4:0] IDEX_rd;
    input [4:0] IFID_rs;
    input IFID_rs_eff;
    input [4:0] IFID_rt;
    input IFID_rt_eff;
    output HU_stall;
    wire match;

//=====Combinational Part=============

    assign match = (IFID_rs_eff & IFID_rs==IDEX_rd) | (IFID_rt_eff & IFID_rt==IDEX_rd);
    assign HU_stall = IDEX_MemRead & match;

endmodule

module BEQBNE(
    ID_opcode,
    ID_funct3,
    ID_is_equal,
    ID_rs,
    ID_rt,
    EX_rd,
    EX_rd_eff,
    BEQBNE_flush, //do branch
    BEQBNE_stall, 
    BEQBNE_branch //ins is branch
);

    //======I/O Declaration================

    input [6:0] ID_opcode;
    input ID_is_equal;
    input [2:0] ID_funct3;
    input [4:0] ID_rs;
    input [4:0] ID_rt;
    input [4:0] EX_rd;
    input EX_rd_eff;
    output BEQBNE_flush;
    output BEQBNE_stall;
    output BEQBNE_branch;
    wire match;

    //=====Combinational Part=============

    assign BEQBNE_flush = BEQBNE_branch & (ID_funct3[0] ^ ID_is_equal);
    assign match = EX_rd==ID_rs | EX_rd==ID_rt;
    assign BEQBNE_stall = match & BEQBNE_branch & EX_rd_eff;
    assign BEQBNE_branch = ID_opcode[6] & ~ID_opcode[2];

endmodule

module JAL(
    ID_opcode,
    JAL_flush
);

    //======I/O Declaration================

    input [6:0] ID_opcode;
    output JAL_flush;

    //=====Combinational Part=============

    assign JAL_flush = ID_opcode[3];

endmodule

module JALR(
    ID_opcode,
    ID_rs,
    EX_rd,
    EX_rd_eff,
    JALR_stall, 
    JALR_flush  //is JALR
);

    //======I/O Declaration===============

    input [6:0] ID_opcode;
    input [4:0] ID_rs;
    input [4:0] EX_rd;
    input EX_rd_eff;
    output JALR_stall;
    output JALR_flush;
    wire match;

    //======Combinational Part============

    assign JALR_flush = ID_opcode[2] & ~ID_opcode[3];
    assign match = ID_rs==EX_rd;
    assign JALR_stall = JALR_flush & match & EX_rd_eff;

endmodule

module JumpBranch(
    ID_opcode,
    ID_funct3,
    ID_rs,
    ID_rt,
    ID_is_equal,
    EX_rd,
    EX_rd_eff,
    ID_Imm,
    ID_PC,
    ID_data1,
    JumpBranch_addr,
    JumpBranch_jump,
    JumpBranch_stall,
    JumpBranch_flush,
    JumpBranch_link,
    JumpBranch_branch,
    JumpBranch_JALR   
);

    //=====I/O Declaration===============

    input [6:0] ID_opcode;
    input [2:0] ID_funct3;
    input [4:0] ID_rs;
    input [4:0] ID_rt;
    input ID_is_equal;
    input [4:0] EX_rd;
    input EX_rd_eff;
    input [31:0] ID_Imm;
    input [31:0] ID_PC;
    input [31:0] ID_data1;   
    output reg [31:0] JumpBranch_addr;
    output JumpBranch_jump;
    output JumpBranch_stall;
    output JumpBranch_flush;
    output JumpBranch_link;
    output JumpBranch_branch;
    output JumpBranch_JALR;
    wire BEQBNE_branch;
    wire BEQBNE_stall;
    wire BEQBNE_flush;
    wire JAL_flush;
    wire JALR_stall;
    wire JALR_flush;   

    wire [31:0] JALR_Addr_Temp;
    //=====Combinational Part===========

    BEQBNE branch(
        .ID_opcode(ID_opcode),
        .ID_funct3(ID_funct3),
        .ID_is_equal(ID_is_equal),
        .ID_rs(ID_rs),
        .ID_rt(ID_rt),
        .EX_rd(EX_rd),
        .EX_rd_eff(EX_rd_eff),
        .BEQBNE_flush(BEQBNE_flush), //do branch
        .BEQBNE_stall(BEQBNE_stall), 
        .BEQBNE_branch(BEQBNE_branch) //ins is branch
    );
    JAL jump(
        .ID_opcode(ID_opcode),
        .JAL_flush(JAL_flush)
    );
    JALR jumpreg(
        .ID_opcode(ID_opcode),
        .ID_rs(ID_rs),
        .EX_rd(EX_rd),
        .EX_rd_eff(EX_rd_eff),
        .JALR_stall(JALR_stall), 
        .JALR_flush(JALR_flush)  //ins is JALR
    );

    assign JumpBranch_flush = BEQBNE_flush | JAL_flush | JALR_flush;
    assign JumpBranch_jump = JumpBranch_flush;
    assign JumpBranch_stall = BEQBNE_stall | JALR_stall;
    assign JumpBranch_link = JAL_flush | JALR_flush;
    assign JumpBranch_branch = BEQBNE_branch;
    assign JumpBranch_JALR = JALR_flush;
    assign JALR_Addr_Temp = ID_data1 + ID_Imm;

    always@(*) begin
        if(JALR_flush)
            JumpBranch_addr = {JALR_Addr_Temp[31:1], 1'b0};
        else 
            JumpBranch_addr = ID_Imm + ID_PC;       
    end

endmodule

module ForwardUnit(
    rs_valid,
    rt_valid,
    in_rs,
    in_rt,
    EXMEM_en,
    EXMEM_rd,
    EXMEM_data,
    MEMWB_en,
    MEMWB_rd,
    MEMWB_data,
    rs_forward,
    rs_forward_data,
    rt_forward,
    rt_forward_data
);
    //==== in/out declaration =================================
    input   [31:0] EXMEM_data, MEMWB_data;
    input   [4:0]  in_rs, in_rt, EXMEM_rd, MEMWB_rd;
    input          rs_valid, rt_valid, EXMEM_en, MEMWB_en;
    output  [31:0] rs_forward_data, rt_forward_data;
    output         rs_forward, rt_forward;

    //==== reg/wire declaration ===============================
    reg     [31:0] rs_forward_data, rt_forward_data;
    reg            rs_forward, rt_forward;

    //==== combinational part =================================
    always @(*) begin
        //for rs
        rs_forward = 0;
        rs_forward_data = 0;
        if (rs_valid & EXMEM_en & (EXMEM_rd==in_rs) & ~(!in_rs)) begin
            rs_forward_data = EXMEM_data;
            rs_forward = 1;
        end
        else begin
            if (rs_valid & MEMWB_en & (MEMWB_rd == in_rs) & ~(!in_rs)) begin
                rs_forward_data = MEMWB_data;
                rs_forward = 1;
            end
        end
        //for rt
        rt_forward = 0;
        rt_forward_data = 0;
        if (rt_valid & EXMEM_en & (EXMEM_rd==in_rt) & ~(!in_rt)) begin
            rt_forward_data = EXMEM_data;
            rt_forward = 1;
        end
        else begin
            if (rt_valid & MEMWB_en & (MEMWB_rd == in_rt) & ~(!in_rt)) begin
                rt_forward_data = MEMWB_data;
                rt_forward = 1;
            end
        end
    end
endmodule






module cache_Improve(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
    integer i;
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    

    localparam  COMPARETAG = 2'b00;
    localparam  ALLOCATE    = 2'b01;
    localparam  WRITEBACK   = 2'b10;
//==== wire/reg definition ================================
    // reg     [127:0] cache [0:7];
    // reg     [127:0] n_cache [0:7];
    // reg     [24:0]  tag [0:7];
    // reg     [24:0]  n_tag [0:7];
    // reg     [7:0] valid, n_valid;
    reg     [153:0] cache [0:7];
    reg     [153:0] n_cache [0:7];

    // // for debug
    // reg     valid [0:7];
    // reg     [24:0]tag [0:7];

    // always@(*) begin
    //     for(i = 0; i < 8; i = i + 1) begin
    //         valid[i] = cache[i][153];
    //         tag[i] = cache[i][152:128];
    //     end
    // end

    // // for debug


    
    reg     [1:0]   state, n_state;
    reg             mem_ready_reg;

    reg                 mem_read, n_mem_read;
    reg                 mem_write, n_mem_write;
    reg     [27:0]      mem_addr, n_mem_addr;
    reg     [127:0]     mem_wdata, n_mem_wdata;

    wire    [24:0]  proc_tag;
    wire    [2:0]   proc_block;
    wire    [1:0]   proc_offset;

    wire    [127:0] data_block;
    wire    [24:0] tag_block;
    wire    valid_block;

    wire    [153:0] chosen_block;

    assign proc_tag = proc_addr[29:5];
    assign proc_block = proc_addr[4:2];
    assign proc_offset = proc_addr[1:0];

    assign data_block = {chosen_block[127:0]};
    assign tag_block =  {chosen_block[152:128]};
    assign valid_block ={chosen_block[153]};

    assign chosen_block = cache[proc_block];

//==== combinational circuit ==============================
    assign proc_stall = ~proc_read & ~proc_write ? 0 : tag_block == proc_tag & valid_block ? 0 : 1;
    // READ: check whether the tag in the specific cache is right or not
    //// proc_block means which block processor wants to access
    //assign proc_rdata = proc_stall ? 32'd0 : cache[proc_block][proc_offset*32 +: 32];

    

    assign proc_rdata = data_block[proc_offset *32 +: 32];
    
    
    /* STATE TRANSITION */
    always@(*) begin
        case(state)
            COMPARETAG: begin
                if(proc_stall) begin
                    if(!valid_block) n_state = ALLOCATE;
                    else n_state = WRITEBACK;
                end
                else n_state = COMPARETAG;
            end
            ALLOCATE: begin
                if(mem_ready_reg) n_state = COMPARETAG;
                else n_state = ALLOCATE;   
            end
            WRITEBACK: begin
                if(mem_ready_reg) n_state = ALLOCATE;
                else n_state = WRITEBACK;   
            end
            default: n_state = state;
        endcase
    end

    always@(*) begin
        for(i = 0; i < 8; i = i + 1) begin
            n_cache[i] = cache[i];
        end
        n_mem_read = mem_read;
        n_mem_write = mem_write;
        n_mem_addr = mem_addr;
        n_mem_wdata = mem_wdata;

        case (state)
            COMPARETAG: begin
                if(proc_stall) begin
                    if(!valid_block) begin // NEED TO ALLOCATE // unvalid
                        n_mem_write = 0;
                        n_mem_read = 1;
                        n_mem_addr = {proc_tag, proc_block};
                    end
                    else begin // NEED WRITEBACK // valid but different tag
                        n_mem_write = 1;
                        n_mem_read = 0;
                        n_mem_addr = {tag_block, proc_block};
                        n_mem_wdata = data_block;
                    end
                end
                else begin
                    if (!proc_read & proc_write) begin
                        n_cache[proc_block][proc_offset * 32 +: 32] = proc_wdata;
                    end
                    // only take operation @ proc_read = 0, proc_write = 1 when proc_stall = 0
                end
            end
            WRITEBACK: begin
                if(mem_ready_reg) begin // NEED ALLOCATE 
                        n_mem_write = 0;
                        n_mem_read = 1;
                        n_mem_addr = {proc_tag,proc_block};
                end
            end
            ALLOCATE: begin
                if(mem_ready_reg) begin
                    n_cache[proc_block] = {1'b1, proc_tag, mem_rdata};
                end
                if(mem_ready) begin
                    n_mem_read = 0;
                end
            end 
            default: begin
            end
        endcase
    end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        for(i = 0; i < 8; i = i + 1) begin
            cache[i] <= 154'd0;
        end
        // valid <= 8'd0;
        state <= COMPARETAG;
        mem_write <= 0;
        mem_read <= 0;
        mem_addr <= 0;
        mem_wdata <= 0;
        mem_ready_reg <= 0;
    end
    else begin  
        for(i = 0; i < 8; i = i + 1) begin
            cache[i] <= n_cache[i];
            // tag[i] <= n_tag[i];
        end
        // valid <= n_valid;
        state <= n_state;
        mem_write <= n_mem_write;
        mem_read <= n_mem_read;
        mem_addr <= n_mem_addr;
        mem_wdata <= n_mem_wdata;
        mem_ready_reg <= mem_ready;
    end
end

endmodule

module READ_only_cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
    integer i;
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    

    localparam  COMPARETAG = 1'b0;
    localparam  ALLOCATE    = 1'b1;
//==== wire/reg definition ================================
    // reg     [127:0] cache [0:7];
    // reg     [127:0] n_cache [0:7];
    // reg     [24:0]  tag [0:7];
    // reg     [24:0]  n_tag [0:7];
    // reg     [7:0] valid, n_valid;
    
    reg     [153:0] cache [0:7];
    reg     [153:0] n_cache [0:7];

    reg     state, n_state;
    reg     mem_ready_reg;

    reg     mem_read, n_mem_read;
    reg     [27:0] mem_addr, n_mem_addr;

    wire    [24:0] proc_tag;
    wire    [2:0] proc_block;
    wire    [1:0] proc_offset;

    wire    [127:0]     data_block;
    wire    [24:0]      tag_block;
    wire                valid_block;

    wire    [153:0] chosen_block;


    assign proc_tag = proc_addr[29:5];
    assign proc_block = proc_addr[4:2];
    assign proc_offset = proc_addr[1:0];

    assign data_block = {chosen_block[127:0]};
    assign tag_block =  {chosen_block[152:128]};
    assign valid_block ={chosen_block[153]};

    assign chosen_block = cache[proc_block];


    
    
    
    assign mem_wdata = 127'd0;
    assign mem_write = 1'b0;

//==== combinational circuit ==============================
    assign proc_stall = ~proc_read ? 0 : tag_block == proc_tag & valid_block ? 0 : 1;
    // READ: check whether the tag in the specific cache is right or not
    //// proc_block means which block processor wants to access
    //assign proc_rdata = proc_stall ? 32'd0 : cache[proc_block][proc_offset*32 +: 32];
    assign proc_rdata = data_block[proc_offset *32 +: 32];
    
    
    /* STATE TRANSITION */
    always@(*) begin
        case(state)
            COMPARETAG: begin
                if(proc_stall) n_state = ALLOCATE;
                else n_state = COMPARETAG;
            end
            ALLOCATE: begin
                if(mem_ready_reg) n_state = COMPARETAG;
                else n_state = ALLOCATE;   
            end
            default: n_state = state;
        endcase
    end

    always@(*) begin
        for(i = 0; i < 8; i = i + 1) begin
            n_cache[i] = cache[i];
            //n_tag[i] = tag[i];
        end
        //n_valid = valid;
        n_mem_read = mem_read;
        n_mem_addr = mem_addr;

        case (state)
            COMPARETAG: begin
                if(proc_stall) begin
                        n_mem_read = 1;
                        n_mem_addr = {proc_tag, proc_block};
                end
            end
            ALLOCATE: begin
                if(mem_ready_reg) begin
                    n_cache[proc_block] = {1'b1, proc_tag, mem_rdata};
                end
                if(mem_ready) begin
                    //n_mem_write = 0;
                    n_mem_read = 0;
                    // n_mem_addr = 0;
                    // n_mem_wdata = 0;
                end
            end 
            default: begin
            end
        endcase
    end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        for(i = 0; i < 8; i = i + 1) begin
            cache[i] <= 128'd0;
        end
        state <= COMPARETAG;
        mem_read <= 0;
        mem_addr <= 0;
        mem_ready_reg <= 0;
    end
    else begin  
        for(i = 0; i < 8; i = i + 1) begin
            cache[i] <= n_cache[i];
        end
        state <= n_state;
        mem_read <= n_mem_read;
        mem_addr <= n_mem_addr;
        mem_ready_reg <= mem_ready;
    end
end

endmodule




module StallUnit_Normal(
    JumpBranch_stall,
    HU_stall,
    JumpBranch_flush,
    InsCache_stall,
    DataCache_stall,
    IFID_is_forward,
    ID_is_bubble,
    EX_is_bubble,
    MEM_is_lw,
    has_stall,
    has_zero
);
//==== in/out declaration =================================
    input          JumpBranch_stall, HU_stall, JumpBranch_flush, InsCache_stall, DataCache_stall;
    input          IFID_is_forward, ID_is_bubble, EX_is_bubble, MEM_is_lw;
    output  [4:0]  has_stall, has_zero;

//==== reg/wire declaration ===============================
    reg     [4:0]  has_stall, has_zero;
    wire           JBH_stall;

//==== combinational part =================================
    assign JBH_stall = JumpBranch_stall | HU_stall;
    always @(*) begin
        //PC
        has_stall[0] = | {JBH_stall, InsCache_stall, DataCache_stall};
        has_zero[0] = 0;

        //IFID
        has_stall[1] =| {JBH_stall, InsCache_stall, DataCache_stall};
        has_zero[1] = JumpBranch_flush | InsCache_stall;

        //IDEX
        has_stall[2] = | {DataCache_stall, InsCache_stall};
        has_zero[2] = JBH_stall;

        //EXMEM
        has_stall[3] = has_stall[2];
        has_zero[3] = 0;

        //MEMWB
        has_stall[4] = has_stall[2];
        has_zero[4] = 0;

    end
endmodule