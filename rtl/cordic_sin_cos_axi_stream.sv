// 2024 Taras Zaporozhets <zaporozhets.taras@gmail.com>
// TODO: Add paddind to s_axis_tdata and m_axis_tdata to make byte alligned data width
`resetall  //
`timescale 1ns / 1ps  //
`default_nettype none  //

module cordic_sin_cos_axi_stream #(
    // Range = 8-24
    parameter integer PHASE_DW  = 16,
    // Range = 8-24
    parameter integer CORDIC_DW = 16
) (
    input  wire                   aclk,
    input  wire                   aresetn,
    // AXI Stream Slave Interface
    input  wire [   PHASE_DW-1:0] s_axis_tdata,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,
    // AXI Stream Master Interface
    output reg  [2*CORDIC_DW-1:0] m_axis_tdata,
    output reg                    m_axis_tvalid,
    output reg                    m_axis_tlast,
    input  wire                   m_axis_tready
);

  // 1.64676025812 =~ system gain
  localparam integer XFscale = 1 << (CORDIC_DW);

  // When the tool calculates the X value for different phase widths, we
  // get rounding errors for every width in the interval [8;24].
  // Depending on the width thess errors cause overflows or smaller amplitudes
  // of the sine waves.
  // The error is not linear nor proportional with the phase. To fix the issue
  // a simple aproximation was chosen.
  localparam integer AproxDwGainError = (CORDIC_DW < 21) ? 4 : (CORDIC_DW <= 24) ? 7 : 0;
  // ((2^N)/2)/1.647...
  localparam integer XValue = ((XFscale / 2) / (1.64676)) - AproxDwGainError;

  localparam integer Iterations = PHASE_DW;

  // CORDIC angle table
  wire [PHASE_DW-1:0] atan_table[CORDIC_DW-1];
  // Internal signals
  reg signed [CORDIC_DW-1:0] x[Iterations];
  reg signed [CORDIC_DW-1:0] y[Iterations];
  reg signed [PHASE_DW-1:0] z[Iterations];
  reg tlast[Iterations];
  reg tvalid[Iterations];



  assign s_axis_tready = m_axis_tready;

  wire [1:0] quadrant = s_axis_tdata[PHASE_DW-1:PHASE_DW-2];

  integer i;
  always @(posedge aclk) begin
    if (!aresetn) begin

      for (i = 0; i < Iterations; i = i + 1) begin
        x[i] <= 0;
        y[i] <= 0;
        z[i] <= 0;
        tlast[i] <= 0;
        tvalid[i] <= 0;
      end
    end else begin
      // Input stage
      if (s_axis_tready) begin
        case (quadrant)
          2'b00, 2'b11: begin
            x[0] <= XValue;
            y[0] <= 0;
            // no changes needed for these quadrants
            z[0] <= s_axis_tdata;
          end

          2'b01: begin
            x[0] <= 0;
            y[0] <= XValue;
            // subtract pi/2 for angle in this quadrant
            z[0] <= {2'b00, s_axis_tdata[PHASE_DW-3:0]};
          end

          2'b10: begin
            x[0] <= 0;
            y[0] <= -XValue;
            // add pi/2 to angles in this quadrant
            z[0] <= {2'b11, s_axis_tdata[PHASE_DW-3:0]};
          end
          default: begin
            // handle any other cases here
            // to suppress: Explicitly define a default case for every case statement.
            // [Style: case-statements] [case-missing-default] warning
          end
        endcase

        tlast[0]  <= s_axis_tlast;
        tvalid[0] <= s_axis_tvalid;
      end
    end
  end

  genvar pipe_i;
  generate
    for (pipe_i = 0; pipe_i < (Iterations - 1); pipe_i = pipe_i + 1) begin : gen_pipeline
      wire z_sign;
      wire signed [CORDIC_DW:0] x_shr, y_shr;

      assign x_shr  = x[pipe_i] >>> pipe_i;  // signed shift right
      assign y_shr  = y[pipe_i] >>> pipe_i;

      //the sign of the current rotation angle
      assign z_sign = z[pipe_i][PHASE_DW-1];

      always @(posedge aclk) begin
        if (~aresetn) begin
          m_axis_tdata  <= 0;
          m_axis_tlast  <= 0;
          m_axis_tvalid <= 0;
        end else begin
          if (m_axis_tready) begin
            if (z_sign) begin
              x[pipe_i+1] <= x[pipe_i] + y_shr;
              y[pipe_i+1] <= y[pipe_i] - x_shr;
              z[pipe_i+1] <= z[pipe_i] + atan_table[pipe_i];
            end else begin
              x[pipe_i+1] <= x[pipe_i] - y_shr;
              y[pipe_i+1] <= y[pipe_i] + x_shr;
              z[pipe_i+1] <= z[pipe_i] - atan_table[pipe_i];
            end
            tlast[pipe_i+1]  <= tlast[pipe_i];
            tvalid[pipe_i+1] <= tvalid[pipe_i];

          end
          m_axis_tdata  <= {x[Iterations-1], y[Iterations-1]};
          m_axis_tvalid <= tvalid[Iterations-1];
          m_axis_tlast  <= tlast[Iterations-1];
        end
      end
    end
  endgenerate

  // Precomputed arctan values in radians
  generate
    if (PHASE_DW == 24) begin : gen_atan_24
      assign atan_table[0]  = 24'd2097152;  // 45.0000000000000
      assign atan_table[1]  = 24'd1238021;  // 26.5650511770780
      assign atan_table[2]  = 24'd654136;  // 14.0362434679265
      assign atan_table[3]  = 24'd332050;  // 7.12501634890180
      assign atan_table[4]  = 24'd166669;  // 3.57633437499735
      assign atan_table[5]  = 24'd83416;  // 1.78991060824607
      assign atan_table[6]  = 24'd41718;  // 0.89517371021107
      assign atan_table[7]  = 24'd20860;  // 0.44761417086055
      assign atan_table[8]  = 24'd10430;  // 0.22381050036853
      assign atan_table[9]  = 24'd5215;  // 0.11190567706620
      assign atan_table[10] = 24'd2608;  // 0.05595289189380
      assign atan_table[11] = 24'd1304;  // 0.02797645261700
      assign atan_table[12] = 24'd652;  // 0.01398822714226
      assign atan_table[13] = 24'd326;  // 0.00699411367535
      assign atan_table[14] = 24'd163;  // 0.00349705685070
      assign atan_table[15] = 24'd81;  // 0.00174852842698
      assign atan_table[16] = 24'd41;  // 0.00087426421369
      assign atan_table[17] = 24'd20;  // 0.00043713210687
      assign atan_table[18] = 24'd10;  // 0.00021856605343
      assign atan_table[19] = 24'd5;  // 0.00010928302672
      assign atan_table[20] = 24'd3;  // 0.00005464151336
      assign atan_table[21] = 24'd1;  // 0.00002732075668
      assign atan_table[22] = 24'd1;  // 0.00001366037834
    end else if (PHASE_DW == 23) begin : gen_atan_23
      assign atan_table[0]  = 23'd1048576;  // 45.0000000000000
      assign atan_table[1]  = 23'd619011;  // 26.5650511770780
      assign atan_table[2]  = 23'd327068;  // 14.0362434679265
      assign atan_table[3]  = 23'd166025;  // 7.12501634890180
      assign atan_table[4]  = 23'd83335;  // 3.57633437499735
      assign atan_table[5]  = 23'd41708;  // 1.78991060824607
      assign atan_table[6]  = 23'd20859;  // 0.89517371021107
      assign atan_table[7]  = 23'd10430;  // 0.44761417086055
      assign atan_table[8]  = 23'd5215;  // 0.22381050036853
      assign atan_table[9]  = 23'd2608;  // 0.11190567706620
      assign atan_table[10] = 23'd1304;  // 0.05595289189380
      assign atan_table[11] = 23'd652;  // 0.02797645261700
      assign atan_table[12] = 23'd326;  // 0.01398822714226
      assign atan_table[13] = 23'd163;  // 0.00699411367535
      assign atan_table[14] = 23'd81;  // 0.00349705685070
      assign atan_table[15] = 23'd41;  // 0.00174852842698
      assign atan_table[16] = 23'd20;  // 0.00087426421369
      assign atan_table[17] = 23'd10;  // 0.00043713210687
      assign atan_table[18] = 23'd5;  // 0.00021856605343
      assign atan_table[19] = 23'd3;  // 0.00010928302672
      assign atan_table[20] = 23'd1;  // 0.00005464151336
      assign atan_table[21] = 23'd1;  // 0.00002732075668
    end else if (PHASE_DW == 22) begin : gen_atan_22
      assign atan_table[0]  = 22'd524288;  // ...
      assign atan_table[1]  = 22'd309505;
      assign atan_table[2]  = 22'd163534;
      assign atan_table[3]  = 22'd83012;
      assign atan_table[4]  = 22'd41667;
      assign atan_table[5]  = 22'd20854;
      assign atan_table[6]  = 22'd10430;
      assign atan_table[7]  = 22'd5215;
      assign atan_table[8]  = 22'd2608;
      assign atan_table[9]  = 22'd1304;
      assign atan_table[10] = 22'd652;
      assign atan_table[11] = 22'd326;
      assign atan_table[12] = 22'd163;
      assign atan_table[13] = 22'd81;
      assign atan_table[14] = 22'd41;
      assign atan_table[15] = 22'd20;
      assign atan_table[16] = 22'd10;
      assign atan_table[17] = 22'd5;
      assign atan_table[18] = 22'd3;
      assign atan_table[19] = 22'd1;
      assign atan_table[20] = 22'd1;
    end else if (PHASE_DW == 21) begin : gen_atan_21
      assign atan_table[0]  = 21'd262144;
      assign atan_table[1]  = 21'd154753;
      assign atan_table[2]  = 21'd81767;
      assign atan_table[3]  = 21'd41506;
      assign atan_table[4]  = 21'd20834;
      assign atan_table[5]  = 21'd10427;
      assign atan_table[6]  = 21'd5215;
      assign atan_table[7]  = 21'd2608;
      assign atan_table[8]  = 21'd1304;
      assign atan_table[9]  = 21'd652;
      assign atan_table[10] = 21'd326;
      assign atan_table[11] = 21'd163;
      assign atan_table[12] = 21'd81;
      assign atan_table[13] = 21'd41;
      assign atan_table[14] = 21'd20;
      assign atan_table[15] = 21'd10;
      assign atan_table[16] = 21'd5;
      assign atan_table[17] = 21'd3;
      assign atan_table[18] = 21'd1;
      assign atan_table[19] = 21'd1;
    end else if (PHASE_DW == 20) begin : gen_atan_20
      assign atan_table[0]  = 20'd131072;
      assign atan_table[1]  = 20'd77376;
      assign atan_table[2]  = 20'd40884;
      assign atan_table[3]  = 20'd20753;
      assign atan_table[4]  = 20'd10417;
      assign atan_table[5]  = 20'd5213;
      assign atan_table[6]  = 20'd2607;
      assign atan_table[7]  = 20'd1304;
      assign atan_table[8]  = 20'd652;
      assign atan_table[9]  = 20'd326;
      assign atan_table[10] = 20'd163;
      assign atan_table[11] = 20'd81;
      assign atan_table[12] = 20'd41;
      assign atan_table[13] = 20'd20;
      assign atan_table[14] = 20'd10;
      assign atan_table[15] = 20'd5;
      assign atan_table[16] = 20'd3;
      assign atan_table[17] = 20'd1;
      assign atan_table[18] = 20'd1;
    end else if (PHASE_DW == 19) begin : gen_atan_19
      assign atan_table[0]  = 19'd65536;
      assign atan_table[1]  = 19'd38688;
      assign atan_table[2]  = 19'd20442;
      assign atan_table[3]  = 19'd10377;
      assign atan_table[4]  = 19'd5208;
      assign atan_table[5]  = 19'd2607;
      assign atan_table[6]  = 19'd1304;
      assign atan_table[7]  = 19'd652;
      assign atan_table[8]  = 19'd326;
      assign atan_table[9]  = 19'd163;
      assign atan_table[10] = 19'd81;
      assign atan_table[11] = 19'd41;
      assign atan_table[12] = 19'd20;
      assign atan_table[13] = 19'd10;
      assign atan_table[14] = 19'd5;
      assign atan_table[15] = 19'd3;
      assign atan_table[16] = 19'd1;
      assign atan_table[17] = 19'd1;
    end else if (PHASE_DW == 18) begin : gen_atan_18
      assign atan_table[0]  = 18'd32768;
      assign atan_table[1]  = 18'd19344;
      assign atan_table[2]  = 18'd10221;
      assign atan_table[3]  = 18'd5188;
      assign atan_table[4]  = 18'd2604;
      assign atan_table[5]  = 18'd1303;
      assign atan_table[6]  = 18'd652;
      assign atan_table[7]  = 18'd326;
      assign atan_table[8]  = 18'd163;
      assign atan_table[9]  = 18'd81;
      assign atan_table[10] = 18'd41;
      assign atan_table[11] = 18'd20;
      assign atan_table[12] = 18'd10;
      assign atan_table[13] = 18'd5;
      assign atan_table[14] = 18'd3;
      assign atan_table[15] = 18'd1;
      assign atan_table[16] = 18'd1;
    end else if (PHASE_DW == 17) begin : gen_atan_17
      assign atan_table[0]  = 17'd16384;
      assign atan_table[1]  = 17'd9672;
      assign atan_table[2]  = 17'd5110;
      assign atan_table[3]  = 17'd2594;
      assign atan_table[4]  = 17'd1302;
      assign atan_table[5]  = 17'd652;
      assign atan_table[6]  = 17'd326;
      assign atan_table[7]  = 17'd163;
      assign atan_table[8]  = 17'd81;
      assign atan_table[9]  = 17'd41;
      assign atan_table[10] = 17'd20;
      assign atan_table[11] = 17'd10;
      assign atan_table[12] = 17'd5;
      assign atan_table[13] = 17'd3;
      assign atan_table[14] = 17'd1;
      assign atan_table[15] = 17'd1;
    end else if (PHASE_DW == 16) begin : gen_atan_16
      assign atan_table[0]  = 16'd8192;
      assign atan_table[1]  = 16'd4836;
      assign atan_table[2]  = 16'd2555;
      assign atan_table[3]  = 16'd1297;
      assign atan_table[4]  = 16'd651;
      assign atan_table[5]  = 16'd326;
      assign atan_table[6]  = 16'd163;
      assign atan_table[7]  = 16'd81;
      assign atan_table[8]  = 16'd41;
      assign atan_table[9]  = 16'd20;
      assign atan_table[10] = 16'd10;
      assign atan_table[11] = 16'd5;
      assign atan_table[12] = 16'd3;
      assign atan_table[13] = 16'd1;
      assign atan_table[14] = 16'd1;
    end else if (PHASE_DW == 15) begin : gen_atan_15
      assign atan_table[0]  = 15'd4096;
      assign atan_table[1]  = 15'd2418;
      assign atan_table[2]  = 15'd1278;
      assign atan_table[3]  = 15'd649;
      assign atan_table[4]  = 15'd326;
      assign atan_table[5]  = 15'd163;
      assign atan_table[6]  = 15'd81;
      assign atan_table[7]  = 15'd41;
      assign atan_table[8]  = 15'd20;
      assign atan_table[9]  = 15'd10;
      assign atan_table[10] = 15'd5;
      assign atan_table[11] = 15'd3;
      assign atan_table[12] = 15'd1;
      assign atan_table[13] = 15'd1;
    end else if (PHASE_DW == 14) begin : gen_atan_14
      assign atan_table[0]  = 14'd2048;
      assign atan_table[1]  = 14'd1209;
      assign atan_table[2]  = 14'd639;
      assign atan_table[3]  = 14'd324;
      assign atan_table[4]  = 14'd163;
      assign atan_table[5]  = 14'd81;
      assign atan_table[6]  = 14'd41;
      assign atan_table[7]  = 14'd20;
      assign atan_table[8]  = 14'd10;
      assign atan_table[9]  = 14'd5;
      assign atan_table[10] = 14'd3;
      assign atan_table[11] = 14'd1;
      assign atan_table[12] = 14'd1;
    end else if (PHASE_DW == 13) begin : gen_atan_13
      assign atan_table[0]  = 13'd1024;
      assign atan_table[1]  = 13'd605;
      assign atan_table[2]  = 13'd319;
      assign atan_table[3]  = 13'd162;
      assign atan_table[4]  = 13'd81;
      assign atan_table[5]  = 13'd41;
      assign atan_table[6]  = 13'd20;
      assign atan_table[7]  = 13'd10;
      assign atan_table[8]  = 13'd5;
      assign atan_table[9]  = 13'd3;
      assign atan_table[10] = 13'd1;
      assign atan_table[11] = 13'd1;
    end else if (PHASE_DW == 12) begin : gen_atan_12
      assign atan_table[0]  = 12'd512;
      assign atan_table[1]  = 12'd302;
      assign atan_table[2]  = 12'd160;
      assign atan_table[3]  = 12'd81;
      assign atan_table[4]  = 12'd41;
      assign atan_table[5]  = 12'd20;
      assign atan_table[6]  = 12'd10;
      assign atan_table[7]  = 12'd5;
      assign atan_table[8]  = 12'd3;
      assign atan_table[9]  = 12'd1;
      assign atan_table[10] = 12'd1;
    end else if (PHASE_DW == 11) begin : gen_atan_11
      assign atan_table[0] = 11'd256;
      assign atan_table[1] = 11'd151;
      assign atan_table[2] = 11'd80;
      assign atan_table[3] = 11'd41;
      assign atan_table[4] = 11'd20;
      assign atan_table[5] = 11'd10;
      assign atan_table[6] = 11'd5;
      assign atan_table[7] = 11'd3;
      assign atan_table[8] = 11'd1;
      assign atan_table[9] = 11'd1;
    end else if (PHASE_DW == 10) begin : gen_atan_10
      assign atan_table[0] = 10'd128;
      assign atan_table[1] = 10'd76;
      assign atan_table[2] = 10'd40;
      assign atan_table[3] = 10'd20;
      assign atan_table[4] = 10'd10;
      assign atan_table[5] = 10'd5;
      assign atan_table[6] = 10'd3;
      assign atan_table[7] = 10'd1;
      assign atan_table[8] = 10'd1;
    end else if (PHASE_DW == 9) begin : gen_atan_9
      assign atan_table[0] = 9'd64;
      assign atan_table[1] = 9'd38;
      assign atan_table[2] = 9'd20;
      assign atan_table[3] = 9'd10;
      assign atan_table[4] = 9'd5;
      assign atan_table[5] = 9'd3;
      assign atan_table[6] = 9'd1;
      assign atan_table[7] = 9'd1;
    end else if (PHASE_DW == 8) begin : gen_atan_8
      assign atan_table[0] = 8'd32;
      assign atan_table[1] = 8'd19;
      assign atan_table[2] = 8'd10;
      assign atan_table[3] = 8'd5;
      assign atan_table[4] = 8'd3;
      assign atan_table[5] = 8'd1;
      assign atan_table[6] = 8'd1;
    end
  endgenerate

endmodule


`resetall  //
