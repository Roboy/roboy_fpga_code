set REL= .\test_bench

vlib work

vcom fpupack.vhd
vcom pre_norm_addsub.vhd
vcom addsub_28.vhd
vcom post_norm_addsub.vhd
vcom pre_norm_mul.vhd
vcom mul_24.vhd
vcom serial_mul.vhd
vcom post_norm_mul.vhd
vcom pre_norm_div.vhd
vcom serial_div.vhd
vcom post_norm_div.vhd
vcom pre_norm_sqrt.vhd
vcom sqrt.vhd
vcom post_norm_sqrt.vhd
vcom comppack.vhd
vcom fpu.vhd

rem *** compile FPU II . Only for simulation!
vcom %REL%\FPU_II\*.*

vcom %REL%\tb_fpu.vhd



pause Start simulation?


vsim -do fpu_wave.do tb_fpu


