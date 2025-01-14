// 2025 Taras Zaporozhets <zaporozhets.taras@gmail.com>
// TODO: Add paddind to s_axis_tdata and m_axis_tdata to make byte alligned data width
`resetall  //
`timescale 1ns / 1ps  //
`default_nettype none  //

module cordic_angle_mag_axi_stream #(
    // Range = 8-24
    parameter integer CORDIC_DW = 16
) (
    input  wire                   aclk,
    input  wire                   aresetn,
    // AXI Stream Slave Interface
    input  wire [2*CORDIC_DW-1:0] s_axis_tdata,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,
    input  wire                   s_axis_tlast,
    // AXI Stream Master Interface
    output reg  [2*CORDIC_DW-1:0] m_axis_tdata,
    output reg                    m_axis_tvalid,
    output reg                    m_axis_tlast,
    input  wire                   m_axis_tready
);
  // minus one because of pre-rotation
  localparam integer Iterations = CORDIC_DW - 1;

  localparam integer WorkingWidth = CORDIC_DW + 2;

  // 
  localparam integer PipelineStages = Iterations + 1;
  // arctan table
  wire [WorkingWidth-1:0] atan_table[Iterations+1];


  reg signed [WorkingWidth-1:0] x_pipeline[PipelineStages];
  reg signed [WorkingWidth-1:0] y_pipeline[PipelineStages];
  reg signed [WorkingWidth-1:0] z_pipeline[PipelineStages];

  reg tlast_pipeline[PipelineStages];
  reg tvalid_pipeline[PipelineStages];

  assign s_axis_tready = m_axis_tready;

  wire signed [CORDIC_DW-1:0] x_in = signed'(s_axis_tdata[    CORDIC_DW-1:0        ]);
  wire signed [CORDIC_DW-1:0] y_in = signed'(s_axis_tdata[2 * CORDIC_DW-1:CORDIC_DW]);

  wire signed [WorkingWidth-1:0] extended_x_in = x_in;
  wire signed [WorkingWidth-1:0] extended_y_in = y_in;

  // Input stage
  integer j;
  always @(posedge aclk) begin
    if (!aresetn) begin
      for (j = 0; j < PipelineStages; j = j + 1) begin
        x_pipeline[j] <= 0;
        y_pipeline[j] <= 0;
        z_pipeline[j] <= 0;
        tlast_pipeline[j] <= 0;
        tvalid_pipeline[j] <= 0;
      end
    end else begin
      if (s_axis_tready) begin
        // Pre-rotation
        case({extended_x_in[WorkingWidth-1], extended_y_in[WorkingWidth-1]})
          2'b01: begin // Quadrant IV
            // Rotate by -315 degrees
            x_pipeline[0] <=  extended_x_in - extended_y_in;
            y_pipeline[0] <=  extended_x_in + extended_y_in;
            z_pipeline[0] <= atan_table[0] * 7;
          end
          2'b11: begin // Quadrant III
            // Rotate by -225 degrees
            x_pipeline[0] <= -extended_x_in - extended_y_in;
            y_pipeline[0] <=  extended_x_in - extended_y_in;
            z_pipeline[0] <= atan_table[0] * 5;
          end
          2'b10: begin // Quadrant II
            // Rotate by -135 degrees
            x_pipeline[0] <= -extended_x_in + extended_y_in;
            y_pipeline[0] <= -extended_x_in - extended_y_in;
            z_pipeline[0] <= atan_table[0] * 3;
          end
          default: begin // Quadrant I
            // Rotate by -45 degrees
            x_pipeline[0] <=  extended_x_in + extended_y_in;
            y_pipeline[0] <= -extended_x_in + extended_y_in;
            z_pipeline[0] <= atan_table[0] * 1;
          end
        endcase

        tlast_pipeline[0]  <= s_axis_tlast;
        tvalid_pipeline[0] <= s_axis_tvalid;
      end
    end
  end

  // Pipeline
  genvar i;
  generate
    for (i = 0; i < Iterations; i = i + 1) begin : gen_pipeline
      always @(posedge aclk) begin
        if (~aresetn) begin

        end else begin
          if (m_axis_tready) begin
            if (y_pipeline[i]  < 0) begin
                x_pipeline[i+1] <= x_pipeline[i] - (y_pipeline[i] >>> (i+1));
                y_pipeline[i+1] <= y_pipeline[i] + (x_pipeline[i] >>> (i+1));
                z_pipeline[i+1] <= z_pipeline[i] - atan_table[i+1];
            end else begin
                x_pipeline[i+1] <= x_pipeline[i] + (y_pipeline[i] >>> (i+1));
                y_pipeline[i+1] <= y_pipeline[i] - (x_pipeline[i] >>> (i+1));
                z_pipeline[i+1] <= z_pipeline[i] + atan_table[i+1];
            end
            tlast_pipeline[i+1]  <= tlast_pipeline[i];
            tvalid_pipeline[i+1] <= tvalid_pipeline[i];
          end
        end
      end
    end
  endgenerate
  
  // Output stage
  wire [CORDIC_DW-1:0] result_magnitude = 16'(x_pipeline[PipelineStages-1][WorkingWidth-3:0]);
  wire [CORDIC_DW-1:0] result_angle     = 16'(z_pipeline[PipelineStages-1][WorkingWidth-1:3]);
  wire                 result_tvalid    = tvalid_pipeline[PipelineStages-1];
  wire                 result_tlast     = tlast_pipeline[PipelineStages-1];

  always @(posedge aclk) begin
    if (~aresetn) begin
        m_axis_tdata  <= 0;
        m_axis_tlast  <= 0;
        m_axis_tvalid <= 0;
    end else begin
        m_axis_tdata  <= {result_magnitude, result_angle};
        m_axis_tvalid <= result_tvalid;
        m_axis_tlast  <= result_tlast;
    end
  end

  // Precomputed arctan values in radians
  generate
    if (CORDIC_DW == 16) begin
      assign atan_table[0]  = 18'd32768;  // 45
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
    end
  endgenerate

endmodule


`resetall  //
