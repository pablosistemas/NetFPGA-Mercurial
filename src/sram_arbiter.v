///////////////////////////////////////////////////////////////////////////////
// $Id: sram_arbiter.v 5697 2009-06-17 22:32:11Z tyabe $
//
// Module: sram_arbiter.v
// Project: NF2.1 openflow switch
// Description: SRAM controller
// Author: Jad Naous <jnaous@stanford.edu>
//
// Provides access to the SRAM for lookups.
// On the first cycle, the rd_0_ack signal is pulsed. After that, we do 16 reads,
// one registers access, one clear counters if necessary, 6 reads,
// 1 write, 6 reads, 1 write.
//
// The register access is tailored to meet the OpenFlow design requirements:
// - When the counters are read, they are reset automatically except for
//   the timestamp.
//
// Licensing: In addition to the NetFPGA license, the following license applies
//            to the source code in the OpenFlow Switch implementation on NetFPGA.
//
// Copyright (c) 2008 The Board of Trustees of The Leland Stanford Junior University
//
// We are making the OpenFlow specification and associated documentation (Software)
// available for public use and benefit with the expectation that others will use,
// modify and enhance the Software and contribute those enhancements back to the
// community. However, since we would like to make the Software available for
// broadest use, with as few restrictions as possible permission is hereby granted,
// free of charge, to any person obtaining a copy of this Software to deal in the
// Software under the copyrights without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// The name and trademarks of copyright holder(s) may NOT be used in advertising
// or publicity pertaining to the Software or any derivatives without specific,
// written prior permission.
///////////////////////////////////////////////////////////////////////////////

`timescale  1ns /  10ps

module sram_arbiter  #(parameter SRAM_ADDR_WIDTH = 19,
                       parameter SRAM_DATA_WIDTH = 36)

   (// register interface
    input                            sram_reg_req,
    input                            sram_reg_rd_wr_L,    // 1 = read, 0 = write
    input [`SRAM_REG_ADDR_WIDTH-1:0] sram_reg_addr,
    input [`CPCI_NF2_DATA_WIDTH-1:0] sram_reg_wr_data,

    output reg                             sram_reg_ack,
    output reg [`CPCI_NF2_DATA_WIDTH -1:0] sram_reg_rd_data,

    // --- Requesters (read and/or write)
    input                            wr_1_req,
    input                            wr_0_req,
    input      [SRAM_ADDR_WIDTH-1:0] wr_0_addr,
    //input      [63:0]                wr_0_addr,
    input      [SRAM_DATA_WIDTH-1:0] wr_0_data,
    output reg                       wr_0_ack,

    input                            rd_1_req,
    input                            rd_0_req,
    input      [SRAM_ADDR_WIDTH-1:0] rd_0_addr, 
    output reg [SRAM_DATA_WIDTH-1:0] rd_0_data,
    //output reg [63:0]                rd_0_data,
    output reg                       rd_0_ack,
    output reg                       rd_0_vld,

    // --- SRAM signals (pins and control)
    output reg [SRAM_ADDR_WIDTH-1:0]   sram_addr1,
    output reg [SRAM_ADDR_WIDTH-1:0]   sram_addr2,
    output reg                         sram_we,
    output reg [SRAM_DATA_WIDTH/9-1:0] sram_bw,
    output reg [SRAM_DATA_WIDTH-1:0]   sram_wr_data,
    input      [SRAM_DATA_WIDTH-1:0]   sram_rd_data,
    output reg                         sram_tri_en,

    // --- Misc

    input reset,
    input clk

    );

   //----------------------- Localparam ----------------------
   localparam  WR_0           =1;
   localparam  RD_0           =2;
   localparam  WR_1           =3;
   localparam  RD_1           =4;
   localparam  REG_INT        =0;
   /*localparam  WAIT           =6;
   localparam  POSWAIT        =7;*/
   localparam  LEN_STATES     =2;
   //----------------------- Localparam ----------------------


   //------------------ Registers/Wires -----------------
   reg                       rd_0_vld_early2, rd_0_vld_early1, rd_0_vld_early3;
   reg [SRAM_DATA_WIDTH-1:0] sram_wr_data_early2, sram_wr_data_early1;
   reg                       sram_tri_en_early2, sram_tri_en_early1;
   reg                       sram_reg_ack_early3, sram_reg_ack_early2, sram_reg_ack_early1;

   reg                       sram_reg_addr_is_high, sram_reg_addr_is_high_d1, sram_reg_addr_is_high_d2;
   reg                       sram_reg_cntr_read;

   wire [SRAM_DATA_WIDTH-1:0] sram_wr_data_early1_shuffled;
   wire [SRAM_DATA_WIDTH-1:0] sram_rd_data_shuffled;
   wire [SRAM_ADDR_WIDTH-1:0] sram_reg_addr_rev;

   //reg                       rd_wr;

   reg                        do_reset;
   reg [1:0]                  state;
   //reg [LEN_STATES-1:0]       last_state;
   //reg [LEN_STATES-1:0]       counter;



   //----------------------- Logic ----------------------
   /* The SRAM considers each byte to be 9bits. So
    * in order to use the sram_bw signals correctly
    * we need to shuffled bits around to match our internal
    * view in the User Data Path that the extra bits
    * are all collected at the MSB of the sram data lines */
   /*generate
      genvar i;
      for (i=0; i<8; i=i+1) begin:gen_sram_data
         assign sram_wr_data_early1_shuffled[i*9 + 8 : i*9] = {sram_wr_data_early1[64 + i], sram_wr_data_early1[i*8 + 7 : i*8]};
         assign {sram_rd_data_shuffled[64 + i], sram_rd_data_shuffled[i*8 + 7 : i*8]} = sram_rd_data[i*9 + 8 : i*9];
      end
   endgenerate*/

   /*genvar i;
   generate
     for (i = 0; i <= SRAM_ADDR_WIDTH; i=i+1) begin
       assign sram_reg_addr_rev[SRAM_ADDR_WIDTH-i] = sram_reg_addr[i];
     end
   endgenerate*/

   
   always @(*) begin
      if(rd_0_req)
            $display("rdaddralways: %x\n",rd_0_addr);
      else if(wr_0_req)
            $display("wraddralways: %x\n",wr_0_addr);
   end
   always @(posedge clk) begin
      if(reset) begin
         state <= REG_INT;
         $display("reset2_sram\n");
      end
      else begin
         state                 <= state + 1'b1;
      end
   end // always @ (posedge clk)

   always @(posedge clk) begin
      if(reset) begin
         //counter               <= 3'h0;
         //state <= REG_INT;
         //last_state <= REG_INT;
         {sram_we, sram_bw}    <= -1;           // active low
         sram_addr1             <= 0;
         sram_addr2             <= 0;
         do_reset              <= 1'b1;
	 // synthesis translate_off
         do_reset              <= 0;
	 // synthesis translate_on
	      //rd_wr                <= 1;
         sram_reg_ack         <= 0;
         rd_0_vld               <= 1'b0;
         $display("reset sram_arbiter\n");
      end

      else begin
         //state                 <= state + 1'b1;
         /*counter                  <= counter + 1'b1;

         // first pipeline stage -- initiate request 
         sram_reg_addr_is_high    <= sram_reg_addr[0];
         sram_reg_cntr_read       <= (((sram_reg_addr[3:0] & 4'he) == `OPENFLOW_EXACT_ENTRY_COUNTERS_POS)
                                      && sram_reg_rd_wr_L
                                      && sram_reg_req);*/
         if(do_reset) begin
            if(sram_addr1 == {SRAM_ADDR_WIDTH{1'b1}}) begin
               do_reset               <= 0;
               {sram_we, sram_bw}     <= -1;           // active low
               rd_0_ack               <= 1'b0;
               rd_0_vld               <= 1'b0;
               wr_0_ack               <= 1'b0;
            end
            else begin
               $display("resetandomemoria\n");
               //each mem have your own addr
               sram_addr1              <= sram_addr1 + 1'b1;
               sram_addr2              <= sram_addr2 + 1'b1;
               {sram_we, sram_bw}     <= 9'h0;
               sram_wr_data_early2    <= 0;
               sram_tri_en_early2     <= 1;
            end // else: !if(sram_addr == {SRAM_ADDR_WIDTH{1'b1}})
         end // if (do_reset)

         else begin
         //first pipeline stage
            sram_reg_addr_is_high <= sram_reg_addr[0];
            if(sram_reg_req) begin
               $display("Reg int\n");
               sram_addr1 <= sram_reg_addr[19:1];
               sram_addr2 <= sram_reg_addr[19:1];
               sram_wr_data_early2 <= sram_reg_addr[0] ? {12'h0,sram_reg_wr_data,36'h0}:{48'h0,sram_reg_wr_data};
               sram_tri_en_early2 <= !sram_reg_rd_wr_L && sram_reg_req;
               if(!sram_reg_rd_wr_L) begin
                  sram_bw <= sram_reg_addr[0] ? 8'h0f : 8'hf0;
                  sram_we <= 1'b0;
               end
               else begin //leitura
                  sram_bw <= 8'hff;
                  sram_we <= 1'b1;
               end 
               rd_0_ack <= 0;
               wr_0_ack <= 0;
               rd_0_vld_early3 <= 0;
               sram_reg_ack_early3 <= sram_reg_req;
            end
            else if(wr_0_req) begin
               $display("WR0\n");
               sram_addr1 <= wr_0_addr;
               sram_addr2 <= wr_0_addr;
               //$display("wraddr: %x, dta: %x\n",wr_0_addr, wr_0_data);
               //sram_wr_data_early2 <= {4'b0,wr_0_data[63:32],4'b0,wr_0_data[31:0]};
               sram_wr_data_early2 <= wr_0_data; //bf use all 72 bits
               sram_tri_en_early2 <= wr_0_req;
               sram_we <= 1'b0;
               //sram_bw <= 8'hf0;
               sram_bw <= 8'h00; //wr_0_req write in two memories
               wr_0_ack <= 1;
               rd_0_ack <= 0;
               rd_0_vld_early3 <= 0;
               sram_reg_ack_early3 <= 0;
            end
            else if(rd_0_req) begin
               $display("RD0\n");
               sram_bw <= 8'hff;
               sram_we <= 1'b1;
               sram_addr1 <= rd_0_addr;
               sram_addr2 <= rd_0_addr;
               $display("rdaddr: %x\n",rd_0_addr);
               sram_tri_en_early2 <= 0;
               rd_0_vld_early3 <= rd_0_req;// || rd_1_req;
               rd_0_ack <= rd_0_req;// || rd_1_req;
               wr_0_ack <= 0;
               sram_reg_ack_early3 <= 0;
            end
            else begin
               wr_0_ack <= 1'b0;
               rd_0_ack <= 1'b0;
               rd_0_vld <= 1'b0;
               {sram_we,sram_bw} <= 9'h1ff;
               rd_0_vld_early3 <= 1'b0;
               sram_tri_en_early2 <= 1'b0;
               sram_wr_data_early2 <= sram_wr_data_early2;
               sram_reg_ack_early3 <= 1'b0;
            end
         end // else: !if(do_reset)
         
         //Second pipeline stage
         sram_tri_en_early1 <= sram_tri_en_early2;
         sram_wr_data_early1 <= sram_wr_data_early2;
         rd_0_vld_early2 <= rd_0_vld_early3;
         sram_reg_ack_early2 <= sram_reg_ack_early3;
         sram_reg_addr_is_high_d1 <= sram_reg_addr_is_high;

         //third pipeline stage - Coloca dado e seta tri_en depois de 2 clocks
         sram_tri_en <= sram_tri_en_early1;
         sram_wr_data <= sram_wr_data_early1;
         //rd_0_data <= {sram_rd_data[71:36],sram_rd_data[31:0]};
         rd_0_vld_early1 <= rd_0_vld_early2;
         sram_reg_ack_early1 <= sram_reg_ack_early2;
         sram_reg_addr_is_high_d2 <= sram_reg_addr_is_high_d1;

         //forth pipeline stage - Coloca dado e seta tri_en depois de 2 clocks
         rd_0_vld <= rd_0_vld_early1;
         sram_reg_ack <= sram_reg_ack_early1;
         //sram_reg_rd_data <= {sram_rd_data[71:36],sram_rd_data[31:0]};
         sram_reg_rd_data <= sram_reg_addr_is_high_d2 ? sram_rd_data[68:36]:sram_rd_data[31:0];
         //rd_0_data <= {sram_rd_data[71:36],sram_rd_data[31:0]};
         rd_0_data <= rd_0_vld_early1?sram_rd_data:rd_0_data; //bf use all 72 bits
      end // else: !if(reset)
   end // always @ (posedge clk)

/************************** Debugging *************************/
   // synthesis translate_off

   // Synthesis code to set the we flag to 0 on startup to remove the annoying
   // "Cypress SRAM stores 'h xxxxxxxxx at addr 'h xxxxx" messages at the
   // beginning of the log file until the clock starts running.
   initial
   begin
      {sram_we, sram_bw} = 9'h1ff;
   end

   // Detect when we write sequential addresses using reg iface, and we skip one
   reg [2:0] seq_state;
   reg [SRAM_ADDR_WIDTH-1:0] prev_addr;
   always @(posedge clk) begin
      if(reset) begin
         seq_state <= 0;
      end
      else begin
         case (seq_state)
            // check when we start a new write
            0: begin
               if(sram_reg_req && !sram_reg_rd_wr_L) begin
                  prev_addr <= sram_reg_addr;
                  seq_state <= 1;
               end
            end

            // wait till req goes low
            1: begin
               if(!sram_reg_req) begin
                  seq_state <= 2;
               end
            end

            // wait for new addr, check if sequential
            2: begin
               if(sram_reg_req && !sram_reg_rd_wr_L) begin
                  // if it is not sequential then we check if we have moved to the next
                  // sequence or if this is a mistake
                  if(sram_reg_addr == prev_addr + 2) begin
                     // Oh oh, we've skipped an address
                     $display("%t %m WARNING: SRAM reg write request skipped an address: %05x.", $time, prev_addr + 1'b1);
                     $stop;
                  end
                  seq_state <= 1;
                  prev_addr <= sram_reg_addr;
               end // if (sram_reg_req && !sram_reg_rd_wr_L)
            end // case: 2
         endcase // case(seq_state)
      end // else: !if(reset)
   end // always @ (posedge clk)

   // synthesis translate_on

endmodule // sram_arbiter


