`include "core.v"

`define LINE [63:0]
`define LINES [16383:0]
`define MEMDELAY 4

module slowmem64(mfc, rdata, addr, wdata, rnotw, strobe, clk);
output reg mfc;
output reg `LINE rdata;
input `LINE addr, wdata;
input rnotw, strobe, clk;
reg [7:0] pend;
reg `LINE raddr;
reg `LINE m `LINES;

initial begin
  pend <= 0;
  readmemh2(m);
  // put your memory initialization code here
end

always @(posedge clk) begin
  if (strobe && rnotw) begin
    // new read request
    raddr <= addr;
    pend <= `MEMDELAY;
  end else begin
    if (strobe && !rnotw) begin
      // do write
      m[addr] <= wdata;
    end

    // pending read?
    if (pend) begin
      // write satisfies pending read
      if ((raddr == addr) && strobe && !rnotw) begin
        rdata <= wdata;
        mfc <= 1;
        pend <= 0;
      end else if (pend == 1) begin
        // finally ready
        rdata <= m[raddr];
        mfc <= 1;
        pend <= 0;
      end else begin
        pend <= pend - 1;
      end
    end else begin
      // return invalid data
      rdata <= 16'hxxxx;
      mfc <= 0;
    end
  end
end
endmodule


// Floating point Verilog modules for CPE480
// Created February 19, 2019 by Henry Dietz, http://aggregate.org/hankd
// Distributed under CC BY 4.0, https://creativecommons.org/licenses/by/4.0/

// Field definitions
`define	INT	signed [15:0]	// integer size
`define FLOAT	[15:0]	// half-precision float size
`define FSIGN	[15]	// sign bit
`define FEXP	[14:7]	// exponent
`define FFRAC	[6:0]	// fractional part (leading 1 implied)

// Constants
`define	FZERO	16'b0	  // float 0
`define F32767  16'h46ff  // closest approx to 32767, actually 32640
`define F32768  16'hc700  // -32768

// Count leading zeros, 16-bit (5-bit result) d=lead0s(s)
module lead0s(d, s);
output wire [4:0] d;
input wire `WORD s;
wire [4:0] t;
wire [7:0] s8;
wire [3:0] s4;
wire [1:0] s2;
assign t[4] = 0;
assign {t[3],s8} = ((|s[15:8]) ? {1'b0,s[15:8]} : {1'b1,s[7:0]});
assign {t[2],s4} = ((|s8[7:4]) ? {1'b0,s8[7:4]} : {1'b1,s8[3:0]});
assign {t[1],s2} = ((|s4[3:2]) ? {1'b0,s4[3:2]} : {1'b1,s4[1:0]});
assign t[0] = !s2[1];
assign d = (s ? t : 16);
endmodule

// Float set-less-than, 16-bit (1-bit result) torf=a<b
module fslt(torf, a, b);
output wire torf;
input wire `FLOAT a, b;
assign torf = (a `FSIGN && !(b `FSIGN)) ||
	      (a `FSIGN && b `FSIGN && (a[14:0] > b[14:0])) ||
	      (!(a `FSIGN) && !(b `FSIGN) && (a[14:0] < b[14:0]));
endmodule

// Floating-point addition, 16-bit r=a+b
module fadd(r, a, b);
output wire `FLOAT r;
input wire `FLOAT a, b;
wire `FLOAT s;
wire [8:0] sexp, sman, sfrac;
wire [7:0] texp, taman, tbman;
wire [4:0] slead;
wire ssign, aegt, amgt, eqsgn;
assign r = ((a == 0) ? b : ((b == 0) ? a : s));
assign aegt = (a `FEXP > b `FEXP);
assign texp = (aegt ? (a `FEXP) : (b `FEXP));
assign taman = (aegt ? {1'b1, (a `FFRAC)} : ({1'b1, (a `FFRAC)} >> (texp - a `FEXP)));
assign tbman = (aegt ? ({1'b1, (b `FFRAC)} >> (texp - b `FEXP)) : {1'b1, (b `FFRAC)});
assign eqsgn = (a `FSIGN == b `FSIGN);
assign amgt = (taman > tbman);
assign sman = (eqsgn ? (taman + tbman) : (amgt ? (taman - tbman) : (tbman - taman)));
lead0s m0(slead, {sman, 7'b0});
assign ssign = (amgt ? (a `FSIGN) : (b `FSIGN));
assign sfrac = sman << slead;
assign sexp = (texp + 1) - slead;
assign s = (sman ? (sexp ? {ssign, sexp[7:0], sfrac[7:1]} : 0) : 0);
endmodule

// Floating-point multiply, 16-bit r=a*b
module fmul(r, a, b);
output wire `FLOAT r;
input wire `FLOAT a, b;
wire [15:0] m; // double the bits in a fraction, we need high bits
wire [7:0] e;
wire s;
assign s = (a `FSIGN ^ b `FSIGN);
assign m = ({1'b1, (a `FFRAC)} * {1'b1, (b `FFRAC)});
assign e = (((a `FEXP) + (b `FEXP)) -127 + m[15]);
assign r = (((a == 0) || (b == 0)) ? 0 : (m[15] ? {s, e, m[14:8]} : {s, e, m[13:7]}));
endmodule

// Floating-point reciprocal, 16-bit r=1.0/a
// Note: requires initialized inverse fraction lookup table
module frecip(r, a);
output wire `FLOAT r;
input wire `FLOAT a;
reg [6:0] look[127:0];
initial $readmemh3(look);
assign r `FSIGN = a `FSIGN;
assign r `FEXP = 253 + (!(a `FFRAC)) - a `FEXP;
assign r `FFRAC = look[a `FFRAC];
endmodule

// Floating-point shift, 16 bit
// Shift +left,-right by integer
module fshift(r, f, i);
output wire `FLOAT r;
input wire `FLOAT f;
input wire `INT i;
assign r `FFRAC = f `FFRAC;
assign r `FSIGN = f `FSIGN;
assign r `FEXP = (f ? (f `FEXP + i) : 0);
endmodule

// Integer to float conversion, 16 bit
module i2f(f, i);
output wire `FLOAT f;
input wire `INT i;
wire [4:0] lead;
wire `WORD pos;
assign pos = (i[15] ? (-i) : i);
lead0s m0(lead, pos);
assign f `FFRAC = (i ? ({pos, 8'b0} >> (16 - lead)) : 0);
assign f `FSIGN = i[15];
assign f `FEXP = (i ? (128 + (14 - lead)) : 0);
endmodule

// Float to integer conversion, 16 bit
// Note: out-of-range values go to -32768 or 32767
module f2i(i, f);
output wire `INT i;
input wire `FLOAT f;
wire `FLOAT ui;
wire tiny, big;
fslt m0(tiny, f, `F32768);
fslt m1(big, `F32767, f);
assign ui = {1'b1, f `FFRAC, 16'b0} >> ((128+22) - f `FEXP);
assign i = (tiny ? 0 : (big ? 32767 : (f `FSIGN ? (-ui) : ui)));
endmodule

`define LOCK_REG    [18:0]
`define LOCK        [18]
`define LOCK_NUM    [17]
`define LOCK_RW     [16]
`define LOCK_ADDR   [15:0]

`define CACHE_LINE  [75:0]
`define CACHE_SIZE  [15:0]

`define CACHE_SHARE         [32:0]
`define CACHE_SHARE_ADDRESS [15:0]
`define CACHE_SHARE_DATA    [31:16]
`define CACHE_SHARE_STROBE  [32]

module L1_cache(share_out, addr, wdata, pass, rnotw, strobe, mfc, rdata, request_status, lock, share_in, clk);
    output `CACHE_SHARE share_out; 
    output reg `LOCK_ADDR addr;
    output reg `LINE wdata;
    output pass, rnotw, strobe;
    input mfc, request_status, lock;
    input `LINE rdata;
    input `CACHE_SHARE share_in;

	reg `CACHE_LINE cache `CACHE_SIZE;

	initial begin
		strobe <= 0;
		pass <= 1;
		rnotw <= 1;
	end
    
	//Waiting on how the connections from the core will be set up
	always @(posedge clk) begin
		if (pass) begin
		//Get new data from core
			//if (instruction is read) begin
				//if (data is not in cache) begin
					//pass <= 0;
					//rnotw <= 1;
					//request rdata from decider
					//tell core to wait
					//end
				//else begin
					//send data from cache to core
				//end
			//end
			//else if (instruction is a write) begin
				//pass <= 0;
				//rnotw <= 0;
				//if (!lock) begin
					//strobe <= 1;
					//send wdata to decider
					//send share_out to other cache
				//end
				//else
					//hold data, tell core to wait
			//end
		end
		else begin //!pass
			if (rnotw) begin
				if (!request_status) begin //not waiting on decider
					if (mfc) begin
						pass <= 1;
						//make new cache line using rdata
						//replace a cache line with new one
						//send data from cache to core
					end
					else begin
						//tell core to wait 
					end
				end
				else begin //waiting on decider
					//tell core to wait
					//resend read request?
				end
			end
			else begin
				if (!lock) begin //wdata hasn't been sent yet
					strobe <= 1;
					//send wdata to decider
					//send share_out to other cache
				end
				else begin
					strobe <= 0;
					if (!request_status) begin //decider sent wdata to slowmem
						pass <= 1;
						//tell core to resume
					//end
					//else begin
						//do nothing?
					//end
				end
			end
		end

		if (share_in`CACHE_SHARE_STROBE) begin
			//if (share_in`CACHE_SHARE_ADDRESS in cache)
				//Update cache entry
		end
	end 

endmodule

`define LOCK_VALUE      [17:0]
`define CACHE_REQUEST   [18:0]
`define PASS_FLAG       [18]
`define CACHE_NUM       [17]
`define CACHE_RW        [16]
`define CACHE_ADDR      [15:0]
`define PASS            1
`define ACCEPT          0

//logic for deciding order of cache reads/writes
module priority_decider(request_to_use, pass, request0_status, request1_status, request0, request1, lock);
    output reg `LOCK_VALUE request_to_use;
    output reg pass, request0_status, request1_status;
    input `CACHE_REQUEST request0, request1;
    input lock;
    
    //logic:
    //only consider the next choices when the lock is undone and ready for a next value
    //if one cache passes, use the other cache unless it is also passing
    //if both caches want to do something, then prioritize reads over writes
    //if both caches want to do the same kind of action, let cache0 go first
    always @(*) begin
        if(!lock) begin
            case({request0 `PASS_FLAG, request1 `PASS_FLAG})
                2'b00: begin
                    case({request0 `CACHE_RW, request1 `CACHE_RW})
                        2'b01: begin
                            request_to_use = request0 `LOCK_VALUE;
                            pass = `ACCEPT;
                            request0_status = `PASS;
                            request1_status = `ACCEPT;
                        end
                        2'b10: begin
                            request_to_use = request1 `LOCK_VALUE;
                            pass = `ACCEPT;
                            request0_status = `ACCEPT;
                            request1_status = `PASS;
                        end
                        default: begin
                            request_to_use = request0 `LOCK_VALUE;
                            pass = `ACCEPT;
                            request0_status = `ACCEPT;
                            request1_status = `PASS;
                        end
                    endcase
                end
                2'b01: begin
                    request_to_use = request0 `LOCK_VALUE;
                    pass = `ACCEPT;
                    request0_status = `ACCEPT;
                    request1_status = `PASS;
                end
                2'b10: begin
                    request_to_use = request1 `LOCK_VALUE;
                    pass = `ACCEPT;
                    request0_status = `PASS;
                    request1_status = `ACCEPT;
                end
                2'b11: begin
                    pass = `PASS;
                    request0_status = `PASS;
                    request1_status = `PASS;
                end
            endcase
        end
    end

endmodule

module processor(halt, reset, clk);
    //lines for slowmem
    reg `LINE rdata, wdata, addr;
    reg strobe, rnotw, mfc, select, pass;
    
    //lock for controling reads/writes
    reg `LOCK_REG lock;
    
    //lines for first L1 cache
    reg `LINE cache0_rdata, cache0_wdata, cache0_addr;
    reg cache0_pass, cache0_strobe, cache0_rnotw, cache0_mfc, cache0_status;

    //lines for second L1 cache
    reg `LINE cache1_rdata, cache1_wdata, cache1_addr;
    reg cache1_pass, cache1_strobe, cache1_rnotw, cache1_mfc, cache1_status;
    
    //lines for sharing between caches
    reg `CACHE_SHARE cache0_1, cache1_0;pass
    
    slowmem64(mfc, rdata, addr, wdata, rnotw, strobe, clk);
    
    //locked when instruction is a read instruction and the mfc has not been set yet
    assign lock `LOCK = !mfc && lock `LOCK_RW;
    
    //logic for memory to use lock
    always @(lock) begin
        addr = lock `LOCK_ADDR;
        rnotw = lock `LOCK_RW;
        select = lock `LOCK_NUM;
        wdata = (!select) ? cache0_wdata : cache1_wdata;
        strobe = ((!select) ? cache0_strobe : cache1_strobe) && pass;
        if(!select) begin
            cache0_rdata = rdata;
        end
        else begin
            cache1_rdata = rdata;
        end
    end
    
    //caches for cores
    L1_cache c0_cache(cache0_1, cache0_addr, cache0_wdata, cache0_pass, cache0_rnotw, cache0_strobe, cache0_status, cache0_rdata, lock `LOCK, cache1_0, clk);
    L1_cache c1_cache(cache1_0, cache1_addr, cache1_wdata, cache1_pass, cache1_rnotw, cache1_strobe, cache1_status, cache1_rdata, lock `LOCK, cache0_1, clk);
    
    priority_decider decider(lock `LOCK_VALUE, pass, cache0_status, cache1_status, {cache0_pass, 1'b0, cache0_rnotw, cache0_addr}, {cache1_pass, 1'b1, cache1_rnotw, cache1_addr}, lock `LOCK);
    
    //core c1(halt, reset, clk, ...);
    //core c2(halt, reset, clk, ...);
    
endmodule

module testbench;
reg reset = 0;
reg clk = 0;
wire halted;
processor PE(halted, reset, clk);
initial begin
  $dumpfile;
  $dumpvars(0, PE); // would normally trace 0, PE
  #10 reset = 1;
  #10 reset = 0;
  while (!halted) begin
    #10 clk = 1;
    #10 clk = 0;
  end
  $finish;
end
endmodule

