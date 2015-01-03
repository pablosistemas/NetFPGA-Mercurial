`timescale  1ns /  10ps

module bloom_filter
   #(
      parameter DATA_WIDTH = 64,
      parameter CTRL_WIDTH = DATA_WIDTH/8,
      parameter SRAM_ADDR_WIDTH = 19, //created
      parameter BITSBUCKET    = 4, //created
      parameter NUMBUCKET  = 12,
      parameter INDEX_LEN = NUMBUCKET*BITSBUCKET,
      parameter UDP_REG_SRC_WIDTH = 2
   )
   (
      input  [DATA_WIDTH-1:0]             in_data,
      input  [CTRL_WIDTH-1:0]             in_ctrl,
      input                               in_wr,
      output                              in_rdy,

      output [DATA_WIDTH-1:0]             out_data,
      output [CTRL_WIDTH-1:0]             out_ctrl,
      output                              out_wr,
      input                               out_rdy,

      output reg                          wr_1_req,
      output reg                          wr_0_req,
      output reg [SRAM_ADDR_WIDTH-1:0]      wr_0_addr,
      output reg [DATA_WIDTH-1:0]         wr_0_data,
      input                               wr_0_ack,

      output reg                          rd_1_req,
      output reg                          rd_0_req,
      output reg [SRAM_ADDR_WIDTH-1:0]      rd_0_addr,
      input [DATA_WIDTH-1:0]              rd_0_data,
      input                               rd_0_ack,
      input                               rd_0_vld,

      // --- Register interface
      input                               reg_req_in,
      input                               reg_ack_in,
      input                               reg_rd_wr_L_in,
      input  [`UDP_REG_ADDR_WIDTH-1:0]    reg_addr_in,
      input  [`CPCI_NF2_DATA_WIDTH-1:0]   reg_data_in,
      input  [UDP_REG_SRC_WIDTH-1:0]      reg_src_in,

      output                              reg_req_out,
      output                              reg_ack_out,
      output                              reg_rd_wr_L_out,
      output  [`UDP_REG_ADDR_WIDTH-1:0]   reg_addr_out,
      output  [`CPCI_NF2_DATA_WIDTH-1:0]  reg_data_out,
      output  [UDP_REG_SRC_WIDTH-1:0]     reg_src_out,

      // misc
      input                                reset,
      input                                clk
   );

   /* ----------Estados----------*/
   localparam SHIFT_RD = 0;
   localparam SHIFT_WR = 1;
   localparam LE_MEM1 = 2;
   localparam LE_MEM2 = 3;
   localparam ATUALIZA_BUCKET_ACK = 4;
   localparam ATUALIZA_BUCKET_DATA = 5;
   localparam BUSCA_BUCKET = 6;
   localparam DECREMENTA_BUCKET = 7;
   localparam INCREMENTA_BUCKET = 8;
   localparam ESCREVE_MEM1 = 9;
   localparam ESCREVE_MEM2 = 10;

   localparam NUM_LEITURAS = 5;
   // Define the log2 function
   `LOG2_FUNC

   function [11:0] buscaff;
      input [71:0] data;
      reg [11:0]  index;
      begin
      index[11] = data[71:71-BITSBUCKET+1]>4'h0;
      index[10] = (~index[11])&data[71-BITSBUCKET:71-2*BITSBUCKET-1]>4'h0;
      index[9] = (~index[10]&~index[11])&data[71-2*BITSBUCKET:71-3*BITSBUCKET-1]>4'h0;
      index[8] = (~index[9]&~index[10]&~index[11])&data[71-3*BITSBUCKET:71-4*BITSBUCKET-1]>4'h0;
      index[7] = (~index[8]&~index[9]&~index[10]&~index[11])&data[71-4*BITSBUCKET:71-5*BITSBUCKET-1]>4'h0;
      index[6] = (~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-5*BITSBUCKET:71-6*BITSBUCKET-1]>4'h0;
      index[5] = (~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-6*BITSBUCKET:71-7*BITSBUCKET-1]>4'h0;
      index[4] = (~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-7*BITSBUCKET:71-8*BITSBUCKET-1]>4'h0;
      index[3] = (~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-8*BITSBUCKET:71-9*BITSBUCKET-1]>4'h0;
      index[2] = (~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-9*BITSBUCKET:71-10*BITSBUCKET-1]>4'h0;
      index[1] = (~index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-10*BITSBUCKET:71-11*BITSBUCKET-1]>4'h0;
      index[0] = (~index[1]&~index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-11*BITSBUCKET:71-12*BITSBUCKET-1]>4'h0;
      buscaff = index; 
   end
   endfunction

   //fifo ctrl
   wire                    in_fifo_empty;
   wire                    in_fifo_data_empty;
   wire                    in_fifo_addr_empty;

   reg                     in_fifo_rd_en;
   reg                     in_fifo_data_rd_en;
   reg                     in_fifo_addr_rd_en;

   wire [2+2*SRAM_ADDR_WIDTH-1:0]             in_fifo_dout;
   wire [72:0]             in_fifo_data_dout;
   wire [SRAM_ADDR_WIDTH-1:0]             in_fifo_addr_dout;
   wire                    in_wr_fifo;
   wire                    in_wr_data_fifo;
   wire                    in_fifo_addr_nearly_full;
   wire                    in_fifo_data_nearly_full;
   wire                    in_fifo_nearly_full;
   reg                     in_wr_addr_fifo;

   reg [21:0]              timer;
   reg [7:0]               bfcur;
   reg signed [8:0]        num_shifts; //signedop (+1bit to 2'comp)
   reg                     data_proc, ack_proc;
   reg                     /*pacote_dados,*/ pacote_ack;
   reg                     /*pacote_dados_next,*/ pacote_ack_next;
   reg                     data_proc_next, ack_proc_next;
   reg [3:0]               state, state_next;
   reg [6:0]               index, index_next;
   reg                     rd_0_req_next,wr_0_req_next;
   reg [SRAM_ADDR_WIDTH-1:0]  rd_0_addr_next, wr_0_addr_next,next_addr_next;
   reg [SRAM_ADDR_WIDTH-1:0]  next_addr,next_addr_wr,next_addr_wr_next;
   reg [SRAM_ADDR_WIDTH-1:0]  addr1, addr1_next;
   reg [SRAM_ADDR_WIDTH-1:0]  addr2, addr2_next;

   wire [SRAM_ADDR_WIDTH-1:0]  hash0, hash1;
   reg [DATA_WIDTH-1:0]        wr_0_data_next;

   reg [2:0]                num_leituras_next, num_leituras;
   reg [INDEX_LEN-1:0]   indice, indice_next;


   assign in_wr_fifo = !in_fifo_nearly_full && (data_pkt||ack_pkt);
   //assign in_wr_fifo = 0;
   assign in_wr_data_fifo = !in_fifo_data_nearly_full && rd_0_vld;

   fallthrough_small_fifo #(.WIDTH(2+2*SRAM_ADDR_WIDTH), .MAX_DEPTH_BITS(3)) input_fifo_req 
        (.din ({data_pkt,ack_pkt,hash0,hash1}),     // Data in
         .wr_en (in_wr_fifo),               // Write enable
         .rd_en (in_fifo_rd_en),       // Read the next word 
         .dout ({in_fifo_dout}),
         .full (),
         .nearly_full (in_fifo_nearly_full),
         .empty (in_fifo_empty),
         .reset (reset),
         .clk (clk));

   fallthrough_small_fifo #(.WIDTH(73), .MAX_DEPTH_BITS(3)) input_fifo_rd_data 
        (.din ({rd_0_vld,rd_0_data}),     // Data in
         .wr_en (in_wr_data_fifo),               // Write enable
         .rd_en (in_fifo_data_rd_en),       // Read the next word 
         .dout ({in_fifo_data_dout}),
         .full (),
         .nearly_full (in_fifo_data_nearly_full),
         .empty (in_fifo_data_empty),
         .reset (reset),
         .clk (clk));

/*   fallthrough_small_fifo #(.WIDTH(SRAM_ADDR_WIDTH), .MAX_DEPTH_BITS(3)) input_fifo_addr 
        (.din ({rd_0_addr_next}),     // Data in
         .wr_en (in_wr_addr_fifo),               // Write enable
         .rd_en (in_fifo_addr_rd_en),       // Read the next word 
         .dout ({in_fifo_addr_dout}),
         .full (),
         .nearly_full (in_fifo_addr_nearly_full),
         .empty (in_fifo_addr_empty),
         .reset (reset),
         .clk (clk));*/

   simulacao #(
        .DATA_WIDTH(DATA_WIDTH),
        .CTRL_WIDTH(CTRL_WIDTH),
        .UDP_REG_SRC_WIDTH (UDP_REG_SRC_WIDTH),
        .SRAM_ADDR_WIDTH(SRAM_ADDR_WIDTH) //added
    ) simulacao (
        .out_data              (out_data),
        .out_ctrl              (out_ctrl),
        .out_wr                (out_wr),
        .out_rdy               (out_rdy),

        .in_data              (/*in_fifo_data*/in_data),
        .in_ctrl              (/*in_fifo_ctrl*/in_ctrl),
        .in_wr                (in_wr),
        .in_rdy               (in_rdy),

        .reg_req_in           (reg_req_in),
        .reg_ack_in           (reg_ack_in),
        .reg_rd_wr_L_in       (reg_rd_wr_L_in),
        .reg_addr_in          (reg_addr_in),
        .reg_data_in          (reg_data_in),
        .reg_src_in           (reg_src_in),

        .reg_req_out           (reg_req_out),
        .reg_ack_out           (reg_ack_out),
        .reg_rd_wr_L_out       (reg_rd_wr_L_out),
        .reg_addr_out          (reg_addr_out),
        .reg_data_out          (reg_data_out),
        .reg_src_out           (reg_src_out),

        .hash_0               (hash0),
        .hash_1               (hash1),
        .data_pkt             (data_pkt),
        .ack_pkt              (ack_pkt),
        .data_proc            (data_proc),
        .ack_proc             (ack_proc),

        .clk              (clk),
        .reset            (reset));

   always@(posedge clk) begin
      //if(timer >= 'b11111110000) begin
      //(int(320*10**-3/12/(8*10**-9)))
      //if(timer >= 'b1100101101110011010101) begin 
      if(timer >= 'b11111110000) begin 
         timer <= 0;
         bfcur <= bfcur + 1;
      end
      else begin
         timer <= timer+1;
         bfcur <= bfcur;
         /*next_addr = {SRAM_ADDR_WIDTH{1'b1}};
         next_addr_wr = {SRAM_ADDR_WIDTH{1'b1}};*/
      end
   end

//initialilly bloom filter was implemented with only 1 hash

   always@(reset) begin
         $display("RESETEDTESTE\n");
         timer = 0;
         wr_0_req = 0;
         rd_0_req = 0;
         in_fifo_rd_en = 0;
         in_fifo_data_rd_en = 0;
         in_fifo_addr_rd_en = 0;
         bfcur = 5;
         num_leituras = 0;
         state = SHIFT_RD;
         next_addr_wr = 0;
         next_addr = 0;
         pacote_ack = 0;
         addr1 = 0;
         addr2 = 0;
   end

   /*always @(posedge clk) begin
      if(rd_0_vld)
         $display("Dadoslidos: %x\n",rd_0_data);
   end*/

   always @(*) begin
      in_fifo_rd_en = 0;
      in_fifo_data_rd_en = 0;
      in_fifo_addr_rd_en = 0;

      state_next = state;
      next_addr_next = next_addr;
      next_addr_wr_next = next_addr_wr;

      rd_0_req_next = 0;
      rd_0_addr_next = rd_0_addr;

      wr_0_req_next = 0;
      wr_0_addr_next = wr_0_addr;
      wr_0_data_next = wr_0_data;

      num_leituras_next = num_leituras;

      //test shifter we answer all reqs without processing
      {data_proc_next,ack_proc_next} = in_fifo_dout[21:20];
      
      pacote_ack_next = pacote_ack;
      //pacote_dados_next = pacote_dados;
      
      addr1_next = addr1;
      addr2_next = addr2;

      indice_next = indice;

      //num_shifts = $signed(bfcur)-$signed(in_fifo_data_dout[23:16]);

      case(state) 
         SHIFT_RD: begin
            $display("SHIFT_RD \n");
            if(!in_fifo_data_nearly_full) begin
               if(num_leituras > NUM_LEITURAS)
                  state_next = SHIFT_WR;
               else begin
                  rd_0_req_next = 1;
                  rd_0_addr_next = next_addr;
                  /*if(next_addr == {{11'b0},{8'b1}})
                     next_addr_next = {SRAM_ADDR_WIDTH{1'b0}};
                  else*/
                  next_addr_next = next_addr + 'h1;
                  num_leituras_next = num_leituras + 'h1;
                  state_next = SHIFT_RD;
               end
            end
            else begin
               state_next = SHIFT_WR;
            end
            //if fifo full we don't generate new reqs 
         end
         SHIFT_WR: begin
            $display("SHIFT_WR\n");
            if(!in_fifo_data_empty) begin
               in_fifo_data_rd_en = 1;
               wr_0_req_next = 1;
               wr_0_addr_next = next_addr_wr;
               wr_0_data_next = {4'b0,in_fifo_data_dout[71:28],bfcur,{next_addr_wr[15:0]}};
               //wr_0_data_next = {4'b0,in_fifo_data_dout[71:28],5'b0,{next_addr_wr}};
               /*if(next_addr_wr == {{11'b0},{8'b1}})
                  next_addr_wr_next = {SRAM_ADDR_WIDTH{1'b0}};
               else*/
               next_addr_wr_next = next_addr_wr + 'h1;
               state_next = SHIFT_WR;
            end
            else begin
               num_leituras_next = 0;
               //state_next = SHIFT_RD;
               state_next = LE_MEM1;
            end
         end
         LE_MEM1 : begin
            $display("LE_MEM1\n");
            indice_next = 48'h0;
            if(!in_fifo_empty) begin
               rd_0_req_next = 1;
               rd_0_addr_next = in_fifo_dout[SRAM_ADDR_WIDTH-1:0];
               addr1_next = in_fifo_dout[SRAM_ADDR_WIDTH-1:0];
               if(in_fifo_dout[SRAM_ADDR_WIDTH-1:0] >= next_addr_wr)
                  $display("endereconaoatualizado: %x\n",in_fifo_dout[SRAM_ADDR_WIDTH-1:0]);
               else
                  $display("enderecoatualizado: %x\n",in_fifo_dout[SRAM_ADDR_WIDTH-1:0]);
               state_next = LE_MEM2;
            end
            else 
               state_next = SHIFT_RD;
         end
         LE_MEM2 : begin
            $display("LE_MEM2\n");
            in_fifo_rd_en = 1;
            rd_0_req_next = 1;
            rd_0_addr_next = in_fifo_dout[2*SRAM_ADDR_WIDTH-1:SRAM_ADDR_WIDTH];
            addr2_next = in_fifo_dout[2*SRAM_ADDR_WIDTH-1:SRAM_ADDR_WIDTH];
            if(in_fifo_dout[SRAM_ADDR_WIDTH-1:0] >= next_addr_wr)
               $display("endereconaoatualizado2: %x\n",in_fifo_dout[2*SRAM_ADDR_WIDTH-1:SRAM_ADDR_WIDTH]);
            else
               $display("enderecoatualizado2: %x\n",in_fifo_dout[2*SRAM_ADDR_WIDTH-1:SRAM_ADDR_WIDTH]);
            if(in_fifo_dout[2+2*SRAM_ADDR_WIDTH-1])
               state_next = ATUALIZA_BUCKET_DATA;
            else
               state_next = ATUALIZA_BUCKET_ACK;
         end
         ATUALIZA_BUCKET_DATA : begin
            $display("ATUALIZA_BUCKET_DATA\n");
            if(!in_fifo_data_empty) begin
               $display("Dadoslidos: %x\n",in_fifo_data_dout[71:0]);
               if(bfcur >= in_fifo_data_dout[23:16])
                  wr_0_data_next[71:24] = {in_fifo_data_dout[71:24]>>($signed(bfcur)-$signed(in_fifo_data_dout[23:16]))};
               else
                  wr_0_data_next[71:24] = {in_fifo_data_dout[71:24]>>($signed(bfcur)+(11-$signed(in_fifo_data_dout[23:16])))};
               wr_0_data_next[23:0] = {bfcur,next_addr_wr[15:0]};
               state_next = INCREMENTA_BUCKET;
            end
            else
               state_next = ATUALIZA_BUCKET_DATA;
         end
         ATUALIZA_BUCKET_ACK : begin
            $display("ATUALIZA_BUCKET_ACK\n");
            if(!in_fifo_data_empty) begin
               $display("Dadoslidos: %x\n",in_fifo_data_dout[71:0]);
               if(bfcur >= in_fifo_data_dout[23:16])
                  wr_0_data_next[71:24] = {in_fifo_data_dout[71:24]>>($signed(bfcur)-$signed(in_fifo_data_dout[23:16]))};
               else
                  wr_0_data_next[71:24] = {in_fifo_data_dout[71:24]>>($signed(bfcur)+(11-$signed(in_fifo_data_dout[23:16])))};
               wr_0_data_next[23:0] = {bfcur,next_addr_wr[15:0]};
               state_next = BUSCA_BUCKET;
            end
            else
               state_next = ATUALIZA_BUCKET_ACK;
         end
         BUSCA_BUCKET : begin
            $display("BUSCA_BUCKET\n");
            {indice_next[44],indice_next[40],indice_next[36],
            indice_next[32],indice_next[28],indice_next[24],
            indice_next[20],indice_next[16],indice_next[12],
            indice_next[8],indice_next[4],indice_next[0]} = buscaff(wr_0_data);
            state_next = DECREMENTA_BUCKET;
         end
         DECREMENTA_BUCKET : begin
            $display("DECREMENTA_BUCKET\n");
            wr_0_data_next = {wr_0_data[71:24]-indice,wr_0_data[23:0]};
            state_next = ESCREVE_MEM1;
         end
         INCREMENTA_BUCKET : begin
            $display("INCREMENTA_BUCKET\n");
            wr_0_data_next = {wr_0_data[71:68]+4'h1,wr_0_data[67:0]};
            state_next = ESCREVE_MEM1;
         end
         ESCREVE_MEM1 : begin
            $display("ESCREVE_MEM1\n");
            wr_0_addr_next = addr1;
            wr_0_req_next = 1;
            state_next = ESCREVE_MEM2;
         end
         ESCREVE_MEM2 : begin
            $display("ESCREVE_MEM2\n");
            wr_0_addr_next = addr2;
            wr_0_req_next = 1;
            state_next = SHIFT_RD;
         end
         default : begin
            $display("DEFAULT\n");
            state_next = SHIFT_RD;
         end
      endcase
   end

   always@(posedge clk) begin
      $display("TESTEBRANCH\n");
      state <= state_next;
      next_addr <= next_addr_next;
      next_addr_wr <= next_addr_wr_next;

      rd_0_req <= rd_0_req_next;
      rd_0_addr <= rd_0_addr_next;

      data_proc <= data_proc_next;
      ack_proc <= ack_proc_next;

      num_leituras <= num_leituras_next;

      wr_0_data <= wr_0_data_next;
      wr_0_req <= wr_0_req_next;
      wr_0_addr <= wr_0_addr_next;

      addr1 <= addr1_next;
      addr2 <= addr2_next;

      pacote_ack <= pacote_ack_next;
      indice <= indice_next;
      //pacote_dados <= pacote_dados_next;
   end //always
endmodule
