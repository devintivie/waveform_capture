`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/05/2020 02:17:15 PM
// Design Name: 
// Module Name: waveform_capture_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module waveform_capture_top
#(
    parameter MsgBitCount=32,
    parameter AddrCmndMsgSize=16,
    parameter DataMsgSize=16,
    parameter SPIAddrSize=15
)
(
    //GPIO
    input wire clk,                 //INPUT CLOCK
//    input wire [3:0]switches,       //ARTY SWITCHES
//    input wire btn0,                //ARTY BTN0
//    input wire btn1,                //ARTY BTN1
//    input wire btn2,                //ARTY BTN2
//    input wire btn3,                //ARTY BTN3
//    output wire [3:0] led,          //ARTY LEDS
    input wire input1,
    output wire output1,
    
    //SPI interface W
    input wire SCK,             //SERIAL CLOCK FROM MASTER
    input wire MOSI,            //MASTER OUT SLAVE IN
    input wire SSEL,            //SLAVE/CHIP SELECT
    output wire MISO_out       //MASTER IN SLAVE OUT
);

//IDENTIFICATION
localparam  PROGRAM_ID = 16'h0100,      //Tells Microcontroller that this a basic spi setup        
            VERSION_LARGE = 12'd1,      //Top 12 bits of Version
            VERSION_SMALL = 4'd1;      //Bottom 4 bits of Version
reg [15:0] ID_reg, VERSION_reg;

localparam MAX_SPI_ADDR = 15'hffff;
wire MISO;          //OUTPUT HOLDER FOR MISO

reg output_reg, output_next;

//ARM
wire rx_cmnd;
wire [(SPIAddrSize-1):0] rx_addr;
reg [(SPIAddrSize-1):0] DPR_Address;
reg [15:0] DPR_DataIn;
reg DPR_Enable;
reg [0:0] DPR_WriteEnable;
wire readwritedata_complete;                 //SUCCESSFULLY READ FROM MASTER(WRITE_DATA) OR WRITE TO MASTER(READ_DATA)
wire addrcmnd_complete;                      //ADDRESS AND COMMAND SUCCESSFULLY READ FROM MASTER
wire [15:0] spi_write_data;
wire [15:0] DPR_DataOut;
reg [15:0] SPI_DataOut;
wire DPRAC_Enable;                            
wire DPRAC_WriteEnable;  

reg [15:0]  output_counter_reg, output_counter_next;
//Clock setup                       
wire clk20MHz;
wire clk100MHz;
clk_wiz_0 clk_gen(
    .clk_in1(clk),
    .clk_out1(clk100MHz),
    .clk_out2(clk20MHz)
);

integer i;
localparam  WAVEFORM_DATA_BASE =  12'h000,
            WAVEFORM_DATA_COUNT = 160;    
localparam  WAVEFORM_DEPTH = 32;
localparam  OUTPUT_DIVIDER = 20;
reg [15:0]  waveform_data_reg[0:WAVEFORM_DATA_COUNT];
reg [15:0]  waveform_data_next[0:WAVEFORM_DATA_COUNT];

reg [(WAVEFORM_DEPTH-1):0] input_data_reg, input_data_next;

//reg [(WAVEFORM_DEPTH-1):0] Inputr;        //REGISTER USED FOR FPGA SYNC

//wire input_rising_edge = (Inputr[1:0] == 2'b01);
//wire input_falling_edge = (Inputr[1:0] == 2'b10);

//SPI SLAVE PI #1
SPI_slave 
#(
.MsgBitCount(MsgBitCount), 
.AddrCmndMsgSize(AddrCmndMsgSize), 
.DataMsgSize(DataMsgSize), 
.AddrSize(SPIAddrSize) 
)
spi_slave_inst(
    .clk(clk100MHz),          //in
    //.reset(reset),      //in
    //SPI SIGNALS
    .SCK(SCK),          //in//SERIAL CLOCK FROM MASTER
    .MOSI(MOSI),        //in//MASTER OUT SLAVE IN
    .SSEL(SSEL),        //in//SLAVE/CHIP SELECT
    .MISO(MISO),        //out//MASTER IN SLAVE OUT
    //DATA TO/FROM DUAL PORT RAM
    .DPR_READ_DATA(SPI_DataOut),       //in//DATA FROM RAM TO WRITE TO MASTER
    .DPR_WRITE_DATA(spi_write_data),    //out//DATA TO WRITE TO RAM DURING WRITE OPERATION
    //CONTROL SIGNALS
    .readwritedata_complete(readwritedata_complete),//out   //SUCCESSFULLY READ FROM MASTER(WRITE_DATA) OR WRITE TO MASTER(READ_DATA)
    .addrcmnd_complete(addrcmnd_complete),          //out   //ADDRESS AND COMMAND SUCCESSFULLY READ FROM MASTER
    .SPI_CommandOut(rx_cmnd),                       //out   //READ/WRITE BIT IN ADDRESS AND COMMAND MESSAGE
    .SPI_AddressOut(rx_addr)                        //out   //READ/WRITE ADDRESS IN ADDRESS AND COMMAND MESSAGE
);

//DUAL PORT RAM CONTROL PI #1
DualPortRamCtrl DPRC(
    .clk(clk100MHz),                                                      //in
    //.reset(reset),                                                  //in
    .SPI_Slave_CommandIn(rx_cmnd),                              //in
    .spi_slave_addrcmnd_complete(addrcmnd_complete),            //in
    .spi_slave_readwritedata_complete(readwritedata_complete),  //in
    .DPR_Enable(DPRAC_Enable),                                 //out
    .DPR_WriteEnable(DPRAC_WriteEnable)                        //out
//    .start()                                                      //out
);

//body
//Sync signals to FPGA clock
always @(posedge clk20MHz)
begin
//    Inputr <= {Inputr[(WAVEFORM_DEPTH-2):0], input1};
    input_data_reg <= input_data_next;
end

always@*
begin
    input_data_next = {input_data_reg[(WAVEFORM_DEPTH-2):0], input1};

end


always@(negedge clk20MHz)
begin
    output_counter_reg <= output_counter_next;
    output_reg <= output_next;
end

always@*
begin
//    output_counter_next = output_counter_reg;
    output_next = output_reg;
    if(output_counter_reg == (OUTPUT_DIVIDER-1))
    begin
        output_counter_next = 16'd0;
    end
    else
    begin
        output_counter_next = output_counter_reg + 1;
    end
    
    if(output_counter_reg < OUTPUT_DIVIDER/2)
    begin
        output_next = 1'b1;
    end
    else
    begin
        output_next = 1'b0;
    end
    
end
       


//REGISTERS LOGIC UPDATE
always @(posedge clk100MHz)
begin
    ID_reg <= PROGRAM_ID;
    VERSION_reg <= {VERSION_LARGE, VERSION_SMALL};
    
    DPR_Address <= rx_addr;
    DPR_DataIn <= spi_write_data;
    DPR_Enable <= DPRAC_Enable;
    DPR_WriteEnable <= DPRAC_WriteEnable; 

    for(i = 0; i < WAVEFORM_DATA_COUNT; i = i + 1)
    begin
        waveform_data_reg[i] <= waveform_data_next[i];
    end
    
    casex(rx_addr)
        MAX_SPI_ADDR - 1 : SPI_DataOut = VERSION_reg;
        MAX_SPI_ADDR : SPI_DataOut = ID_reg;        
        
        WAVEFORM_DATA_BASE + 8'd0 : SPI_DataOut = input_data_reg[15:0];
        WAVEFORM_DATA_BASE + 8'd1 : SPI_DataOut = input_data_reg[31:16];
        WAVEFORM_DATA_BASE + 8'd2 : SPI_DataOut = waveform_data_reg[2];
        WAVEFORM_DATA_BASE + 8'd3 : SPI_DataOut = waveform_data_reg[3];
        WAVEFORM_DATA_BASE + 8'd4 : SPI_DataOut = waveform_data_reg[4];
        WAVEFORM_DATA_BASE + 8'd5 : SPI_DataOut = waveform_data_reg[5];
        WAVEFORM_DATA_BASE + 8'd6 : SPI_DataOut = waveform_data_reg[6];
        WAVEFORM_DATA_BASE + 8'd7 : SPI_DataOut = waveform_data_reg[7];
        WAVEFORM_DATA_BASE + 8'd8 : SPI_DataOut = waveform_data_reg[8];
        WAVEFORM_DATA_BASE + 8'd9 : SPI_DataOut = waveform_data_reg[9];
        WAVEFORM_DATA_BASE + 8'd10 : SPI_DataOut = waveform_data_reg[10];
        WAVEFORM_DATA_BASE + 8'd11 : SPI_DataOut = waveform_data_reg[11];
        WAVEFORM_DATA_BASE + 8'd12 : SPI_DataOut = waveform_data_reg[12];
        WAVEFORM_DATA_BASE + 8'd13 : SPI_DataOut = waveform_data_reg[13];
        WAVEFORM_DATA_BASE + 8'd14 : SPI_DataOut = waveform_data_reg[14];
        WAVEFORM_DATA_BASE + 8'd15 : SPI_DataOut = waveform_data_reg[15];
        WAVEFORM_DATA_BASE + 8'd16 : SPI_DataOut = waveform_data_reg[16];
        WAVEFORM_DATA_BASE + 8'd17 : SPI_DataOut = waveform_data_reg[17];
        WAVEFORM_DATA_BASE + 8'd18 : SPI_DataOut = waveform_data_reg[18];
        WAVEFORM_DATA_BASE + 8'd19 : SPI_DataOut = waveform_data_reg[19];
        WAVEFORM_DATA_BASE + 8'd20 : SPI_DataOut = waveform_data_reg[20];
        WAVEFORM_DATA_BASE + 8'd21 : SPI_DataOut = waveform_data_reg[21];
        WAVEFORM_DATA_BASE + 8'd22 : SPI_DataOut = waveform_data_reg[22];
        WAVEFORM_DATA_BASE + 8'd23 : SPI_DataOut = waveform_data_reg[23];
        WAVEFORM_DATA_BASE + 8'd24 : SPI_DataOut = waveform_data_reg[24];
        WAVEFORM_DATA_BASE + 8'd25 : SPI_DataOut = waveform_data_reg[25];
        WAVEFORM_DATA_BASE + 8'd26 : SPI_DataOut = waveform_data_reg[26];
        WAVEFORM_DATA_BASE + 8'd27 : SPI_DataOut = waveform_data_reg[27];
        WAVEFORM_DATA_BASE + 8'd28 : SPI_DataOut = waveform_data_reg[28];
        WAVEFORM_DATA_BASE + 8'd29 : SPI_DataOut = waveform_data_reg[29];
        WAVEFORM_DATA_BASE + 8'd30 : SPI_DataOut = waveform_data_reg[30];
        WAVEFORM_DATA_BASE + 8'd31 : SPI_DataOut = waveform_data_reg[31];
        WAVEFORM_DATA_BASE + 8'd32 : SPI_DataOut = waveform_data_reg[32];
        WAVEFORM_DATA_BASE + 8'd33 : SPI_DataOut = waveform_data_reg[33];
        WAVEFORM_DATA_BASE + 8'd34 : SPI_DataOut = waveform_data_reg[34];
        WAVEFORM_DATA_BASE + 8'd35 : SPI_DataOut = waveform_data_reg[35];
        WAVEFORM_DATA_BASE + 8'd36 : SPI_DataOut = waveform_data_reg[36];
        WAVEFORM_DATA_BASE + 8'd37 : SPI_DataOut = waveform_data_reg[37];
        WAVEFORM_DATA_BASE + 8'd38 : SPI_DataOut = waveform_data_reg[38];
        WAVEFORM_DATA_BASE + 8'd39 : SPI_DataOut = waveform_data_reg[39];
        WAVEFORM_DATA_BASE + 8'd40 : SPI_DataOut = waveform_data_reg[40];
        WAVEFORM_DATA_BASE + 8'd41 : SPI_DataOut = waveform_data_reg[41];
        WAVEFORM_DATA_BASE + 8'd42 : SPI_DataOut = waveform_data_reg[42];
        WAVEFORM_DATA_BASE + 8'd43 : SPI_DataOut = waveform_data_reg[43];
        WAVEFORM_DATA_BASE + 8'd44 : SPI_DataOut = waveform_data_reg[44];
        WAVEFORM_DATA_BASE + 8'd45 : SPI_DataOut = waveform_data_reg[45];
        WAVEFORM_DATA_BASE + 8'd46 : SPI_DataOut = waveform_data_reg[46];
        WAVEFORM_DATA_BASE + 8'd47 : SPI_DataOut = waveform_data_reg[47];
        WAVEFORM_DATA_BASE + 8'd48 : SPI_DataOut = waveform_data_reg[48];
        WAVEFORM_DATA_BASE + 8'd49 : SPI_DataOut = waveform_data_reg[49];
        WAVEFORM_DATA_BASE + 8'd50 : SPI_DataOut = waveform_data_reg[50];
        WAVEFORM_DATA_BASE + 8'd51 : SPI_DataOut = waveform_data_reg[51];
        WAVEFORM_DATA_BASE + 8'd52 : SPI_DataOut = waveform_data_reg[52];
        WAVEFORM_DATA_BASE + 8'd53 : SPI_DataOut = waveform_data_reg[53];
        WAVEFORM_DATA_BASE + 8'd54 : SPI_DataOut = waveform_data_reg[54];
        WAVEFORM_DATA_BASE + 8'd55 : SPI_DataOut = waveform_data_reg[55];
        WAVEFORM_DATA_BASE + 8'd56 : SPI_DataOut = waveform_data_reg[56];
        WAVEFORM_DATA_BASE + 8'd57 : SPI_DataOut = waveform_data_reg[57];
        WAVEFORM_DATA_BASE + 8'd58 : SPI_DataOut = waveform_data_reg[58];
        WAVEFORM_DATA_BASE + 8'd59 : SPI_DataOut = waveform_data_reg[59];
        WAVEFORM_DATA_BASE + 8'd60 : SPI_DataOut = waveform_data_reg[60];
        WAVEFORM_DATA_BASE + 8'd61 : SPI_DataOut = waveform_data_reg[61];
        WAVEFORM_DATA_BASE + 8'd62 : SPI_DataOut = waveform_data_reg[62];
        WAVEFORM_DATA_BASE + 8'd63 : SPI_DataOut = waveform_data_reg[63];
        WAVEFORM_DATA_BASE + 8'd64 : SPI_DataOut = waveform_data_reg[64];
        WAVEFORM_DATA_BASE + 8'd65 : SPI_DataOut = waveform_data_reg[65];
        WAVEFORM_DATA_BASE + 8'd66 : SPI_DataOut = waveform_data_reg[66];
        WAVEFORM_DATA_BASE + 8'd67 : SPI_DataOut = waveform_data_reg[67];
        WAVEFORM_DATA_BASE + 8'd68 : SPI_DataOut = waveform_data_reg[68];
        WAVEFORM_DATA_BASE + 8'd69 : SPI_DataOut = waveform_data_reg[69];
        WAVEFORM_DATA_BASE + 8'd70 : SPI_DataOut = waveform_data_reg[70];
        WAVEFORM_DATA_BASE + 8'd71 : SPI_DataOut = waveform_data_reg[71];
        WAVEFORM_DATA_BASE + 8'd72 : SPI_DataOut = waveform_data_reg[72];
        WAVEFORM_DATA_BASE + 8'd73 : SPI_DataOut = waveform_data_reg[73];
        WAVEFORM_DATA_BASE + 8'd74 : SPI_DataOut = waveform_data_reg[74];
        WAVEFORM_DATA_BASE + 8'd75 : SPI_DataOut = waveform_data_reg[75];
        WAVEFORM_DATA_BASE + 8'd76 : SPI_DataOut = waveform_data_reg[76];
        WAVEFORM_DATA_BASE + 8'd77 : SPI_DataOut = waveform_data_reg[77];
        WAVEFORM_DATA_BASE + 8'd78 : SPI_DataOut = waveform_data_reg[78];
        WAVEFORM_DATA_BASE + 8'd79 : SPI_DataOut = waveform_data_reg[79];
        WAVEFORM_DATA_BASE + 8'd80 : SPI_DataOut = waveform_data_reg[80];
        WAVEFORM_DATA_BASE + 8'd81 : SPI_DataOut = waveform_data_reg[81];
        WAVEFORM_DATA_BASE + 8'd82 : SPI_DataOut = waveform_data_reg[82];
        WAVEFORM_DATA_BASE + 8'd83 : SPI_DataOut = waveform_data_reg[83];
        WAVEFORM_DATA_BASE + 8'd84 : SPI_DataOut = waveform_data_reg[84];
        WAVEFORM_DATA_BASE + 8'd85 : SPI_DataOut = waveform_data_reg[85];
        WAVEFORM_DATA_BASE + 8'd86 : SPI_DataOut = waveform_data_reg[86];
        WAVEFORM_DATA_BASE + 8'd87 : SPI_DataOut = waveform_data_reg[87];
        WAVEFORM_DATA_BASE + 8'd88 : SPI_DataOut = waveform_data_reg[88];
        WAVEFORM_DATA_BASE + 8'd89 : SPI_DataOut = waveform_data_reg[89];
        WAVEFORM_DATA_BASE + 8'd90 : SPI_DataOut = waveform_data_reg[90];
        WAVEFORM_DATA_BASE + 8'd91 : SPI_DataOut = waveform_data_reg[91];
        WAVEFORM_DATA_BASE + 8'd92 : SPI_DataOut = waveform_data_reg[92];
        WAVEFORM_DATA_BASE + 8'd93 : SPI_DataOut = waveform_data_reg[93];
        WAVEFORM_DATA_BASE + 8'd94 : SPI_DataOut = waveform_data_reg[94];
        WAVEFORM_DATA_BASE + 8'd95 : SPI_DataOut = waveform_data_reg[95];
        WAVEFORM_DATA_BASE + 8'd96 : SPI_DataOut = waveform_data_reg[96];
        WAVEFORM_DATA_BASE + 8'd97 : SPI_DataOut = waveform_data_reg[97];
        WAVEFORM_DATA_BASE + 8'd98 : SPI_DataOut = waveform_data_reg[98];
        WAVEFORM_DATA_BASE + 8'd99 : SPI_DataOut = waveform_data_reg[99];
        WAVEFORM_DATA_BASE + 8'd100 : SPI_DataOut = waveform_data_reg[100];
        WAVEFORM_DATA_BASE + 8'd101 : SPI_DataOut = waveform_data_reg[101];
        WAVEFORM_DATA_BASE + 8'd102 : SPI_DataOut = waveform_data_reg[102];
        WAVEFORM_DATA_BASE + 8'd103 : SPI_DataOut = waveform_data_reg[103];
        WAVEFORM_DATA_BASE + 8'd104 : SPI_DataOut = waveform_data_reg[104];
        WAVEFORM_DATA_BASE + 8'd105 : SPI_DataOut = waveform_data_reg[105];
        WAVEFORM_DATA_BASE + 8'd106 : SPI_DataOut = waveform_data_reg[106];
        WAVEFORM_DATA_BASE + 8'd107 : SPI_DataOut = waveform_data_reg[107];
        WAVEFORM_DATA_BASE + 8'd108 : SPI_DataOut = waveform_data_reg[108];
        WAVEFORM_DATA_BASE + 8'd109 : SPI_DataOut = waveform_data_reg[109];
        WAVEFORM_DATA_BASE + 8'd110 : SPI_DataOut = waveform_data_reg[110];
        WAVEFORM_DATA_BASE + 8'd111 : SPI_DataOut = waveform_data_reg[111];
        WAVEFORM_DATA_BASE + 8'd112 : SPI_DataOut = waveform_data_reg[112];
        WAVEFORM_DATA_BASE + 8'd113 : SPI_DataOut = waveform_data_reg[113];
        WAVEFORM_DATA_BASE + 8'd114 : SPI_DataOut = waveform_data_reg[114];
        WAVEFORM_DATA_BASE + 8'd115 : SPI_DataOut = waveform_data_reg[115];
        WAVEFORM_DATA_BASE + 8'd116 : SPI_DataOut = waveform_data_reg[116];
        WAVEFORM_DATA_BASE + 8'd117 : SPI_DataOut = waveform_data_reg[117];
        WAVEFORM_DATA_BASE + 8'd118 : SPI_DataOut = waveform_data_reg[118];
        WAVEFORM_DATA_BASE + 8'd119 : SPI_DataOut = waveform_data_reg[119];
        WAVEFORM_DATA_BASE + 8'd120 : SPI_DataOut = waveform_data_reg[120];
        WAVEFORM_DATA_BASE + 8'd121 : SPI_DataOut = waveform_data_reg[121];
        WAVEFORM_DATA_BASE + 8'd122 : SPI_DataOut = waveform_data_reg[122];
        WAVEFORM_DATA_BASE + 8'd123 : SPI_DataOut = waveform_data_reg[123];
        WAVEFORM_DATA_BASE + 8'd124 : SPI_DataOut = waveform_data_reg[124];
        WAVEFORM_DATA_BASE + 8'd125 : SPI_DataOut = waveform_data_reg[125];
        WAVEFORM_DATA_BASE + 8'd126 : SPI_DataOut = waveform_data_reg[126];
        WAVEFORM_DATA_BASE + 8'd127 : SPI_DataOut = waveform_data_reg[127];
        WAVEFORM_DATA_BASE + 8'd128 : SPI_DataOut = waveform_data_reg[128];
        WAVEFORM_DATA_BASE + 8'd129 : SPI_DataOut = waveform_data_reg[129];
        WAVEFORM_DATA_BASE + 8'd130 : SPI_DataOut = waveform_data_reg[130];
        WAVEFORM_DATA_BASE + 8'd131 : SPI_DataOut = waveform_data_reg[131];
        WAVEFORM_DATA_BASE + 8'd132 : SPI_DataOut = waveform_data_reg[132];
        WAVEFORM_DATA_BASE + 8'd133 : SPI_DataOut = waveform_data_reg[133];
        WAVEFORM_DATA_BASE + 8'd134 : SPI_DataOut = waveform_data_reg[134];
        WAVEFORM_DATA_BASE + 8'd135 : SPI_DataOut = waveform_data_reg[135];
        WAVEFORM_DATA_BASE + 8'd136 : SPI_DataOut = waveform_data_reg[136];
        WAVEFORM_DATA_BASE + 8'd137 : SPI_DataOut = waveform_data_reg[137];
        WAVEFORM_DATA_BASE + 8'd138 : SPI_DataOut = waveform_data_reg[138];
        WAVEFORM_DATA_BASE + 8'd139 : SPI_DataOut = waveform_data_reg[139];
        WAVEFORM_DATA_BASE + 8'd140 : SPI_DataOut = waveform_data_reg[140];
        WAVEFORM_DATA_BASE + 8'd141 : SPI_DataOut = waveform_data_reg[141];
        WAVEFORM_DATA_BASE + 8'd142 : SPI_DataOut = waveform_data_reg[142];
        WAVEFORM_DATA_BASE + 8'd143 : SPI_DataOut = waveform_data_reg[143];
        WAVEFORM_DATA_BASE + 8'd144 : SPI_DataOut = waveform_data_reg[144];
        WAVEFORM_DATA_BASE + 8'd145 : SPI_DataOut = waveform_data_reg[145];
        WAVEFORM_DATA_BASE + 8'd146 : SPI_DataOut = waveform_data_reg[146];
        WAVEFORM_DATA_BASE + 8'd147 : SPI_DataOut = waveform_data_reg[147];
        WAVEFORM_DATA_BASE + 8'd148 : SPI_DataOut = waveform_data_reg[148];
        WAVEFORM_DATA_BASE + 8'd149 : SPI_DataOut = waveform_data_reg[149];
        WAVEFORM_DATA_BASE + 8'd150 : SPI_DataOut = waveform_data_reg[150];
        WAVEFORM_DATA_BASE + 8'd151 : SPI_DataOut = waveform_data_reg[151];
        WAVEFORM_DATA_BASE + 8'd152 : SPI_DataOut = waveform_data_reg[152];
        WAVEFORM_DATA_BASE + 8'd153 : SPI_DataOut = waveform_data_reg[153];
        WAVEFORM_DATA_BASE + 8'd154 : SPI_DataOut = waveform_data_reg[154];
        WAVEFORM_DATA_BASE + 8'd155 : SPI_DataOut = waveform_data_reg[155];
        WAVEFORM_DATA_BASE + 8'd156 : SPI_DataOut = waveform_data_reg[156];
        WAVEFORM_DATA_BASE + 8'd157 : SPI_DataOut = waveform_data_reg[157];
        WAVEFORM_DATA_BASE + 8'd158 : SPI_DataOut = waveform_data_reg[158];
        WAVEFORM_DATA_BASE + 8'd159 : SPI_DataOut = waveform_data_reg[159];
        
        default: SPI_DataOut = 16'hffff;
    endcase
end
    
//REGISTERS LOGIC STATE CHANGE 
//always@*
//begin
//    for(i = 0; i < WAVEFORM_DATA_COUNT; i = i + 1)
//    begin
//        waveform_data_next[i] = waveform_data_reg[i];
//    end    
    
//    waveform_data_next[0] = Inputr[15:0];
//    waveform_data_next[1] = Inputr[31:16];
//    waveform_data_next[2] = Inputr[47:32];
//    waveform_data_next[3] = Inputr[63:48];
    
//    //Add Global Reset Logic?
//    if(DPR_Enable && DPR_WriteEnable)
//    begin
//        case(DPR_Address)
           
//            WAVEFORM_DATA_BASE + 8'd0 : waveform_data_next[0] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd1 : waveform_data_next[1] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd2 : waveform_data_next[2] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd3 : waveform_data_next[3] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd4 : waveform_data_next[4] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd5 : waveform_data_next[5] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd6 : waveform_data_next[6] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd7 : waveform_data_next[7] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd8 : waveform_data_next[8] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd9 : waveform_data_next[9] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd10 : waveform_data_next[10] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd11 : waveform_data_next[11] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd12 : waveform_data_next[12] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd13 : waveform_data_next[13] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd14 : waveform_data_next[14] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd15 : waveform_data_next[15] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd16 : waveform_data_next[16] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd17 : waveform_data_next[17] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd18 : waveform_data_next[18] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd19 : waveform_data_next[19] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd20 : waveform_data_next[20] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd21 : waveform_data_next[21] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd22 : waveform_data_next[22] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd23 : waveform_data_next[23] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd24 : waveform_data_next[24] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd25 : waveform_data_next[25] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd26 : waveform_data_next[26] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd27 : waveform_data_next[27] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd28 : waveform_data_next[28] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd29 : waveform_data_next[29] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd30 : waveform_data_next[30] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd31 : waveform_data_next[31] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd32 : waveform_data_next[32] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd33 : waveform_data_next[33] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd34 : waveform_data_next[34] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd35 : waveform_data_next[35] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd36 : waveform_data_next[36] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd37 : waveform_data_next[37] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd38 : waveform_data_next[38] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd39 : waveform_data_next[39] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd40 : waveform_data_next[40] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd41 : waveform_data_next[41] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd42 : waveform_data_next[42] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd43 : waveform_data_next[43] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd44 : waveform_data_next[44] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd45 : waveform_data_next[45] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd46 : waveform_data_next[46] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd47 : waveform_data_next[47] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd48 : waveform_data_next[48] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd49 : waveform_data_next[49] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd50 : waveform_data_next[50] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd51 : waveform_data_next[51] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd52 : waveform_data_next[52] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd53 : waveform_data_next[53] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd54 : waveform_data_next[54] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd55 : waveform_data_next[55] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd56 : waveform_data_next[56] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd57 : waveform_data_next[57] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd58 : waveform_data_next[58] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd59 : waveform_data_next[59] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd60 : waveform_data_next[60] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd61 : waveform_data_next[61] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd62 : waveform_data_next[62] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd63 : waveform_data_next[63] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd64 : waveform_data_next[64] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd65 : waveform_data_next[65] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd66 : waveform_data_next[66] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd67 : waveform_data_next[67] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd68 : waveform_data_next[68] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd69 : waveform_data_next[69] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd70 : waveform_data_next[70] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd71 : waveform_data_next[71] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd72 : waveform_data_next[72] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd73 : waveform_data_next[73] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd74 : waveform_data_next[74] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd75 : waveform_data_next[75] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd76 : waveform_data_next[76] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd77 : waveform_data_next[77] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd78 : waveform_data_next[78] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd79 : waveform_data_next[79] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd80 : waveform_data_next[80] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd81 : waveform_data_next[81] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd82 : waveform_data_next[82] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd83 : waveform_data_next[83] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd84 : waveform_data_next[84] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd85 : waveform_data_next[85] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd86 : waveform_data_next[86] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd87 : waveform_data_next[87] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd88 : waveform_data_next[88] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd89 : waveform_data_next[89] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd90 : waveform_data_next[90] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd91 : waveform_data_next[91] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd92 : waveform_data_next[92] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd93 : waveform_data_next[93] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd94 : waveform_data_next[94] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd95 : waveform_data_next[95] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd96 : waveform_data_next[96] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd97 : waveform_data_next[97] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd98 : waveform_data_next[98] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd99 : waveform_data_next[99] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd100 : waveform_data_next[100] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd101 : waveform_data_next[101] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd102 : waveform_data_next[102] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd103 : waveform_data_next[103] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd104 : waveform_data_next[104] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd105 : waveform_data_next[105] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd106 : waveform_data_next[106] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd107 : waveform_data_next[107] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd108 : waveform_data_next[108] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd109 : waveform_data_next[109] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd110 : waveform_data_next[110] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd111 : waveform_data_next[111] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd112 : waveform_data_next[112] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd113 : waveform_data_next[113] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd114 : waveform_data_next[114] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd115 : waveform_data_next[115] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd116 : waveform_data_next[116] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd117 : waveform_data_next[117] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd118 : waveform_data_next[118] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd119 : waveform_data_next[119] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd120 : waveform_data_next[120] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd121 : waveform_data_next[121] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd122 : waveform_data_next[122] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd123 : waveform_data_next[123] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd124 : waveform_data_next[124] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd125 : waveform_data_next[125] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd126 : waveform_data_next[126] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd127 : waveform_data_next[127] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd128 : waveform_data_next[128] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd129 : waveform_data_next[129] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd130 : waveform_data_next[130] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd131 : waveform_data_next[131] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd132 : waveform_data_next[132] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd133 : waveform_data_next[133] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd134 : waveform_data_next[134] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd135 : waveform_data_next[135] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd136 : waveform_data_next[136] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd137 : waveform_data_next[137] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd138 : waveform_data_next[138] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd139 : waveform_data_next[139] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd140 : waveform_data_next[140] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd141 : waveform_data_next[141] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd142 : waveform_data_next[142] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd143 : waveform_data_next[143] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd144 : waveform_data_next[144] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd145 : waveform_data_next[145] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd146 : waveform_data_next[146] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd147 : waveform_data_next[147] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd148 : waveform_data_next[148] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd149 : waveform_data_next[149] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd150 : waveform_data_next[150] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd151 : waveform_data_next[151] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd152 : waveform_data_next[152] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd153 : waveform_data_next[153] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd154 : waveform_data_next[154] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd155 : waveform_data_next[155] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd156 : waveform_data_next[156] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd157 : waveform_data_next[157] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd158 : waveform_data_next[158] = DPR_DataIn;
//            WAVEFORM_DATA_BASE + 8'd159 : waveform_data_next[159] = DPR_DataIn;
//        endcase
//    end
//end

assign MISO_out = MISO;         //CONNECT MISO OUTPUT TO MISO HOLDER
assign output1 = output_reg;


endmodule
