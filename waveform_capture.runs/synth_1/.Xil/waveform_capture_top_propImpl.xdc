set_property SRC_FILE_INFO {cfile:c:/Users/devin/Workspaces/VivadoWorkspace/waveform_capture/waveform_capture.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0/clk_wiz_0_in_context.xdc rfile:../../../waveform_capture.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0/clk_wiz_0_in_context.xdc id:1 order:EARLY scoped_inst:clk_gen} [current_design]
set_property SRC_FILE_INFO {cfile:C:/Users/devin/Workspaces/VivadoWorkspace/waveform_capture/waveform_capture.srcs/constrs_1/new/waveform_capture.xdc rfile:../../../waveform_capture.srcs/constrs_1/new/waveform_capture.xdc id:2} [current_design]
current_instance clk_gen
set_property src_info {type:SCOPED_XDC file:1 line:1 export:INPUT save:INPUT read:READ} [current_design]
create_clock -period 10.000 [get_ports -no_traverse {}]
set_property src_info {type:SCOPED_XDC file:1 line:4 export:INPUT save:INPUT read:READ} [current_design]
create_generated_clock -source [get_ports clk_in1] -edges {1 2 3} -edge_shift {0.000 20.000 40.000} [get_ports {}]
set_property src_info {type:SCOPED_XDC file:1 line:6 export:INPUT save:INPUT read:READ} [current_design]
create_generated_clock -source [get_ports clk_in1] -edges {1 2 3} -edge_shift {0.000 0.000 0.000} [get_ports {}]
current_instance
set_property src_info {type:XDC file:2 line:7 export:INPUT save:INPUT read:READ} [current_design]
set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { clk }]; #IO_L12P_T1_MRCC_35 Sch=gclk[100]
set_property src_info {type:XDC file:2 line:87 export:INPUT save:INPUT read:READ} [current_design]
set_property -dict { PACKAGE_PIN V15   IOSTANDARD LVCMOS33 } [get_ports { input1  }]; #IO_L16P_T2_CSI_B_14          Sch=ck_io[0]
set_property src_info {type:XDC file:2 line:94 export:INPUT save:INPUT read:READ} [current_design]
set_property -dict { PACKAGE_PIN T16   IOSTANDARD LVCMOS33 } [get_ports { output1  }]; #IO_L15N_T2_DQS_DOUT_CSO_B_14 Sch=ck_io[7]
set_property src_info {type:XDC file:2 line:166 export:INPUT save:INPUT read:READ} [current_design]
set_property -dict { PACKAGE_PIN G1    IOSTANDARD LVCMOS33 } [get_ports { MISO_out }]; #IO_L17N_T2_35 Sch=ck_miso
set_property src_info {type:XDC file:2 line:167 export:INPUT save:INPUT read:READ} [current_design]
set_property -dict { PACKAGE_PIN H1    IOSTANDARD LVCMOS33 } [get_ports { MOSI }]; #IO_L17P_T2_35 Sch=ck_mosi
set_property src_info {type:XDC file:2 line:168 export:INPUT save:INPUT read:READ} [current_design]
set_property -dict { PACKAGE_PIN F1    IOSTANDARD LVCMOS33 } [get_ports { SCK }]; #IO_L18P_T2_35 Sch=ck_sck
set_property src_info {type:XDC file:2 line:169 export:INPUT save:INPUT read:READ} [current_design]
set_property -dict { PACKAGE_PIN C1    IOSTANDARD LVCMOS33 } [get_ports { SSEL }]; #IO_L16N_T2_35 Sch=ck_ss
