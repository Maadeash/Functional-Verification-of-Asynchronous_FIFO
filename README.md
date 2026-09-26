# Functional Verification of Asynchronous FIFO With CDC

---

## Aim  
To perform **functional verification** of an **Asynchronous FIFO (First-In-First-Out)** design using **SystemVerilog/UVM**, ensuring reliable data transfer between two clock domains.

---

## Apparatus Required  
- Computer with **Windows OS**  
- **Synopsys vcs**  

---

## Description  
An **Asynchronous FIFO** allows data to be safely transferred between two independent clock domains — typically used in systems where producer and consumer operate at different clock frequencies.  

Functional verification ensures that:
- The FIFO **correctly stores and retrieves data**.  
- **Full** and **Empty** flags operate correctly.  
- No **data corruption** occurs despite differing clock speeds.  

This experiment verifies FIFO behavior through **random stimulus generation** and **testbench-driven verification** in SystemVerilog.

---

## Features  
- Asynchronous FIFO design with independent read/write clocks  
- Verification using **SystemVerilog testbench**  
- Includes **randomized data generation**  
- Compatible with **Synopsys vcs** or **EDA Playground**

---

## SystemVerilog Code

### Asynchronous FIFO Design (`async_fifo.sv`)
```
module asyn_fifo #(parameter depth=4'd4,width=4'd8)(
  input rst,
  input wr_en,rd_en,
  input [width-1:0]din,
  input wr_clk,rd_clk,
  output reg full,
  output reg empty,
  output reg [width-1:0]dout
);
  parameter addr_width=3;
  reg [width-1:0]mem[0:depth-1];
  reg [addr_width-1:0]wr_ptr_bin,rd_ptr_bin;
  reg [addr_width-1:0]wr_ptr_gray,rd_ptr_gray;
  reg [addr_width-1:0]wr_ptr_bin_nxt,rd_ptr_bin_nxt;
  reg [addr_width-1:0]wr_ptr_gray_nxt,rd_ptr_gray_nxt;

  reg [addr_width-1:0]wr_ptr_gray_sync1,rd_ptr_gray_sync1;
  reg [addr_width-1:0]wr_ptr_gray_sync2,rd_ptr_gray_sync2;

  //future write pointer logic
  always@(*) begin
      wr_ptr_bin_nxt=wr_ptr_bin;
      wr_ptr_gray_nxt=wr_ptr_gray;
      if(wr_en) begin
      wr_ptr_bin_nxt=wr_ptr_bin+1;
      wr_ptr_gray_nxt=(wr_ptr_bin_nxt>>1)^wr_ptr_bin_nxt;
     end
  end
  
  //future read pointer logic
  always@(*) begin
      rd_ptr_bin_nxt=rd_ptr_bin;
      rd_ptr_gray_nxt=rd_ptr_gray;
      if(rd_en) begin
      rd_ptr_bin_nxt=rd_ptr_bin+1;
      rd_ptr_gray_nxt=(rd_ptr_bin_nxt>>1)^rd_ptr_bin_nxt;
     end
  end
  
  //write logic
  always@(posedge wr_clk or posedge rst) begin
    if(rst) begin
      wr_ptr_bin<=0;
      wr_ptr_gray<=0;
    end
    else begin
      if(wr_en && !full) begin
        mem[wr_ptr_bin[addr_width-2:0]]<=din;
        wr_ptr_bin<=wr_ptr_bin_nxt;
        wr_ptr_gray<=wr_ptr_gray_nxt;
      end
    end
  end
  
  //read logic
  always@(posedge rd_clk or posedge rst) begin
    if(rst) begin
      rd_ptr_bin<=0;
      rd_ptr_gray<=0;
    end
    else begin
      if(!empty && rd_en) begin
        dout<=mem[rd_ptr_bin[addr_width-2:0]];
        rd_ptr_bin<=rd_ptr_bin_nxt;
        rd_ptr_gray<=rd_ptr_gray_nxt;
      end
    end
  end
  
  //write cdc
  always@(posedge wr_clk or posedge rst) begin
    if(rst) begin
      rd_ptr_gray_sync1<=0;
      rd_ptr_gray_sync2<=0;
    end
    else begin
      rd_ptr_gray_sync1<=rd_ptr_gray;
      rd_ptr_gray_sync2<=rd_ptr_gray_sync1;
    end
  end
  
  //read cdc
  always@(posedge rd_clk or posedge rst) begin
    if(rst) begin
      wr_ptr_gray_sync1<=0;
      wr_ptr_gray_sync2<=0;
    end
    else begin
      wr_ptr_gray_sync1<=wr_ptr_gray;
      wr_ptr_gray_sync2<=wr_ptr_gray_sync1;
    end
  end
  
    // empty logic — registered on rd_clk
  always@(posedge rd_clk or posedge rst) begin
    if(rst)
      empty <= 1'b1;
    else
      empty <= (rd_ptr_gray_nxt == wr_ptr_gray_sync2);
  end

  // full logic — registered on wr_clk
  always@(posedge wr_clk or posedge rst) begin
    if(rst)
      full <= 1'b0;
    else
      full <= (wr_ptr_gray_nxt == {~rd_ptr_gray_sync2[2:1], rd_ptr_gray_sync2[0]});
  end
endmodule
```
### Testbench
```
`include "uvm_macros.svh"
import uvm_pkg::*;

interface vf #(width=8,depth=4)(input logic wr_clk,rd_clk,rst);
  logic wr_en,rd_en;
  logic [width-1:0]din;
  logic full,empty;
  logic [width-1:0]dout;
endinterface

class transaction #(parameter width=8,depth=4) extends uvm_sequence_item;
  `uvm_object_param_utils(transaction#(width,depth))
  rand bit wr_en,rd_en;
  rand bit [width-1:0]din;
  bit full,empty;
  bit [width-1:0]dout;
  constraint cg_in{din<8'd100;}
  function new(string name="transaction");
    super.new(name);
  endfunction
endclass

class generator #(parameter width=8,depth=4) extends uvm_sequence#(transaction#(width,depth));
  `uvm_object_param_utils(generator#(width,depth))
  transaction #(width,depth) tr;
  function new(string name="generator");
    super.new(name);
  endfunction
  task body();
    repeat(20) begin
      tr=transaction#(width,depth)::type_id::create("tr");
      start_item(tr);
      tr.randomize();
      finish_item(tr);
      $display("Generated Values:%d  %d  %d",tr.din,tr.wr_en,tr.rd_en);
    end
    $display("Generator Completed");
  endtask
endclass

class driver extends uvm_driver#(transaction);
  `uvm_component_utils(driver)
  mailbox gen2drv_wr, gen2drv_rd;
  transaction tr, tr_wr, tr_rd;
  virtual vf vif;
  function new(string name="driver", uvm_component parent=null);
    super.new(name,parent);
  endfunction
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    gen2drv_wr=new();
    gen2drv_rd=new();
    if(!uvm_config_db#(virtual vf)::get(this,"","vif",vif))
      `uvm_fatal("DRV","vif not found")
  endfunction
  task run_phase(uvm_phase phase);
    fork
      get_items();
      drive_wr();
      drive_rd();
    join
  endtask
  task get_items();
    repeat(20) begin
      seq_item_port.get_next_item(tr);
      gen2drv_wr.put(tr);
      gen2drv_rd.put(tr);
      seq_item_port.item_done();
    end
  endtask
  task drive_wr();
    repeat(20) begin
      gen2drv_wr.get(tr_wr);
      @(negedge vif.wr_clk);
      vif.wr_en<=tr_wr.wr_en;
      vif.din<=tr_wr.din;
      $display("Driver WR:%d  %d",tr_wr.din,tr_wr.wr_en);
    end
  endtask
  task drive_rd();
    repeat(20) begin
      gen2drv_rd.get(tr_rd);
      @(negedge vif.rd_clk);
      vif.rd_en<=tr_rd.rd_en;
      $display("Driver RD:%d",tr_rd.rd_en);
    end
  endtask
endclass

class monitor extends uvm_monitor;
  `uvm_component_utils(monitor)
  transaction t_wr,t_rd;
  uvm_analysis_port #(transaction) mon2sb_wr, mon2sb_rd;
  virtual vf vif;
  covergroup fifo_cov;
	  cg_rst: coverpoint vif.rst{
		bins low_rst={0};
		bins high_rst={1};
	  }
  endgroup
  
  covergroup cg_wr;
		write: coverpoint t_wr.wr_en{
			bins low={0};
			bins high={1};
		}
        cg_din: coverpoint t_wr.din{
		bins low_din={[0:33]};
		bins med_din={[34:66]};
		bins high_din={[67:99]};
	  }
      wr_din_cross: cross write,cg_din;
  endgroup

  covergroup cg_rd;
		read: coverpoint t_rd.rd_en{
			bins low={0};
            bins high={1};
	}
  endgroup

  function new(string name="monitor", uvm_component parent=null);
    super.new(name,parent);
    fifo_cov=new();
    cg_wr=new();
	cg_rd=new();
  endfunction
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    mon2sb_wr=new("mon2sb_wr",this);
    mon2sb_rd=new("mon2sb_rd",this);
    if(!uvm_config_db#(virtual vf)::get(this,"","vif",vif))
      `uvm_fatal("MON","vif not found")
  endfunction
  task run_phase(uvm_phase phase);
    fork
      mon_wr();
      mon_rd();
    join
  endtask
  task mon_wr();
    repeat(20) begin
      t_wr=transaction::type_id::create("t_wr");
      @(negedge vif.wr_clk);
      #1;
      t_wr.wr_en=vif.wr_en;
      t_wr.din=vif.din;
      t_wr.full=vif.full;
      
      fifo_cov.sample();
      cg_wr.sample();
      mon2sb_wr.write(t_wr);
      $display("Monitor WR:%d  %d",t_wr.din,t_wr.wr_en);
    end
  endtask
  task mon_rd();
    transaction prev;
    prev=null;
    repeat(20) begin
      t_rd=transaction::type_id::create("t_rd");
      @(negedge vif.rd_clk);
      #1;
      t_rd.rd_en=vif.rd_en;
      t_rd.empty=vif.empty;
      if(prev!=null) begin
        prev.dout=vif.dout;
        mon2sb_rd.write(prev);
        $display("Monitor RD:%d  dout=%d",prev.rd_en,prev.dout);
      end
      prev=t_rd;
      cg_rd.sample();
    end
    @(negedge vif.rd_clk);
    #1;
    prev.dout=vif.dout;
    mon2sb_rd.write(prev);
    cg_rd.sample();
    $display("Monitor RD:%d  dout=%d",prev.rd_en,prev.dout);
  endtask
endclass

class scoreboard #(width=8,depth=4) extends uvm_scoreboard;
  `uvm_component_param_utils(scoreboard#(width,depth))
  uvm_tlm_analysis_fifo #(transaction) mon2sb_wr, mon2sb_rd;
  bit [width-1:0] expected_dout[$];
  function new(string name="scoreboard", uvm_component parent=null);
    super.new(name,parent);
  endfunction
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    mon2sb_wr=new("mon2sb_wr",this);
    mon2sb_rd=new("mon2sb_rd",this);
  endfunction
  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    fork
      handle_wr();
      handle_rd();
    join
    phase.drop_objection(this);
  endtask
  task handle_wr();
    transaction t;
    repeat(20) begin
      mon2sb_wr.get(t);
      if(t.wr_en==1 && !t.full) begin
        expected_dout.push_back(t.din);
        $display("Actual Value when write:%d",t.din);
      end
    end
  endtask
  task handle_rd();
    transaction t;
    bit [width-1:0] expected;
    bit [width-1:0]p_count;
    bit [width-1:0]f_count;
    repeat(20) begin
      mon2sb_rd.get(t);
      if(t.rd_en==1 && !t.empty && expected_dout.size()>0) begin
        expected=expected_dout.pop_front();
        $display("expected:%d == actual:%d",expected,t.dout);
        if(expected==t.dout) begin $display("---PASS---"); p_count=p_count+1; end
        else begin $display("---FAIL---"); f_count=f_count+1; end
      end
      else
        $display("SKIP");
    end
    $display("------Overall Pass Count == %d-------",p_count);
    $display("------Overall Fail Count == %d-------",f_count);
  endtask
endclass

class environment extends uvm_env;
  `uvm_component_utils(environment)
  uvm_sequencer #(transaction) seqr;
  driver drv;
  monitor mon;
  scoreboard scb;
  function new(string name="environment", uvm_component parent=null);
    super.new(name,parent);
  endfunction
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    seqr=uvm_sequencer#(transaction)::type_id::create("seqr",this);
    drv=driver::type_id::create("drv",this);
    mon=monitor::type_id::create("mon",this);
    scb=scoreboard#()::type_id::create("scb",this);
  endfunction
  function void connect_phase(uvm_phase phase);
    drv.seq_item_port.connect(seqr.seq_item_export);
    mon.mon2sb_wr.connect(scb.mon2sb_wr.analysis_export);
    mon.mon2sb_rd.connect(scb.mon2sb_rd.analysis_export);
  endfunction
endclass

class test extends uvm_test;
  `uvm_component_utils(test)
  environment env;
  generator gen;
  function new(string name="test", uvm_component parent=null);
    super.new(name,parent);
  endfunction
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env=environment::type_id::create("env",this);
  endfunction
  task run_phase(uvm_phase phase);
    phase.raise_objection(this);
    gen=generator#()::type_id::create("gen");
    gen.start(env.seqr);
    phase.drop_objection(this);
  endtask
  function void report_phase(uvm_phase phase);
    $display("All tests Completed");
    $display("Write enable coverage == %0.2f%%",env.mon.cg_wr.write.get_coverage());
    $display("Read enable coverage  == %0.2f%%",env.mon.cg_rd.read.get_coverage());
    $display("Data input coverage   == %0.2f%%",env.mon.cg_wr.cg_din.get_coverage());
    $display("Reset coverage        == %0.2f%%",env.mon.fifo_cov.cg_rst.get_coverage());
    $display("--------------------------");
    $display("Total Functional coverage == %0.2f%%",env.mon.fifo_cov.get_coverage());
    $display("--------------------------");
  endfunction
endclass

module tb_top;
  logic wr_clk,rd_clk,rst;
  vf vif(wr_clk,rd_clk,rst); 
  asyn_fifo dut(.rst(vif.rst),.wr_en(vif.wr_en),.rd_en(vif.rd_en),.din(vif.din),.wr_clk(vif.wr_clk),.rd_clk(vif.rd_clk),.full(vif.full),.empty(vif.empty),.dout(vif.dout));
  always #5 wr_clk=~wr_clk;
  always #7 rd_clk=~rd_clk;
  initial begin
    uvm_config_db#(virtual vf)::set(null,"*","vif",vif);
    run_test("test");
  end
  initial begin
    wr_clk=0;
    rd_clk=0;
    rst=1;
    #20;
    rst=0;
    #5000;
    $finish;
  end
endmodule
```
### Simulation Output

<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/c0b3419e-0f6a-427b-9d6c-6bad85cbce9a" />


### Result

The Functional Verification of Asynchronous FIFO was successfully carried out using SystemVerilog.The FIFO was verified for correct data transfer across two asynchronous clock domains, ensuring proper write/read synchronization, flag operation, and data integrity.
