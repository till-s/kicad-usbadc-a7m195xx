# ULPI clock

create_clock -period 16.665 -name ulpiClk [get_ports ulpiClk]

# PLL Clock
create_clock -period  9.615 -name pllClk  [get_ports pllClk]

# we use the MMCM to create a negative phase shift to compensate
# for the internal clock delay. This works fine
# for input paths but in the opposite direction the worst
# case found by the timer is (a bit less than) 1 cycle off.
# Remedy with a multicycle path

set_multicycle_path -from [get_clocks ulpiClk] 2

# The ulpiDir -> ulpiDat[x] combinatorial path (which switches the output drivers
# off) does not use the delayed clock; must relax the hold timing.
# It would be desirable to not use a multicycle path at all for this combinatorial
# path but on the low-end device we must allow 2 cycles for turn-around which
# is in-principle OK but may result in a bit of driver fighting.
set_multicycle_path -hold -from [get_clocks ulpiClk] -to [get_clocks ulpiClk] 1

# 130MHz
#set adcPeriod   7.69
# 120MHz
set adcPeriod   8.333
# min guaranteed by datasheet
set adcSetupMin 1.3
# min guaranteed by datasheet
set adcHoldMin  0.7
# it's' not entirely clear how these scale with
# the sampling frequency. The datasheet says
# (using the default timing) that the dclk edge
# leads the data edge by 1/8 period. Thus, we assume
#  Thold   =   T/8 - hold_margin
#  Tsetup  = 3 T/8 - setup margin
#  hold_margin   = 1/fsmax/8 - Thold_min  = 0.915 - 0.7 = 0.2615
#  setup_margin  = 3/8/fsmax - Tsetup_min = 2.885 - 1.3 = 1.585

set adcHold    [expr 1.0/8.0*${adcPeriod} - 0.2615]
set adcSetup   [expr 3.0/8.0*${adcPeriod} - 1.585 ]
set adcLoDC    [expr ${adcPeriod}*0.38]
set adcHiDC    [expr ${adcPeriod}*0.62]
set adcMdDC    [expr ${adcPeriod}*0.50]

# ADC Clocks with min/max duty cycle
create_clock      -period ${adcPeriod} -name adcDClkLoDC -waveform "0.0 ${adcLoDC}" [get_ports adcDClk]
create_clock -add -period ${adcPeriod} -name adcDClkHiDC -waveform "0.0 ${adcHiDC}" [get_ports adcDClk]
create_clock -add -period ${adcPeriod} -name adcDClk                                [get_ports adcDClk]

set_clock_groups -group [get_clocks -include_generated_clocks adcDClkLoDC] -group [get_clocks -include_generated_clocks adcDClkHiDC] -group [get_clocks -include_generated_clocks adcDClk] -physically_exclusive

set_input_delay -add_delay -max -clock [get_clocks adcDClkLoDC]             [expr ${adcLoDC} - ${adcSetup}] [get_ports adc*DDR*]
set_input_delay -add_delay -max -clock [get_clocks adcDClkLoDC] -clock_fall [expr ${adcHiDC} - ${adcSetup}] [get_ports adc*DDR*]

set_input_delay -add_delay -max -clock [get_clocks adcDClkHiDC]             [expr ${adcHiDC} - ${adcSetup}] [get_ports adc*DDR*]
set_input_delay -add_delay -max -clock [get_clocks adcDClkHiDC] -clock_fall [expr ${adcLoDC} - ${adcSetup}] [get_ports adc*DDR*]

set_input_delay -add_delay -max -clock [get_clocks adcDClk]                 [expr ${adcMdDC} - ${adcSetup}] [get_ports adc*DDR*]
set_input_delay -add_delay -max -clock [get_clocks adcDClk] -clock_fall     [expr ${adcMdDC} - ${adcSetup}] [get_ports adc*DDR*]

set_input_delay -add_delay -min -clock [get_clocks adcDClkLoDC]             ${adcHold}                      [get_ports adc*DDR*]
set_input_delay -add_delay -min -clock [get_clocks adcDClkLoDC] -clock_fall ${adcHold}                      [get_ports adc*DDR*]
set_input_delay -add_delay -min -clock [get_clocks adcDClkHiDC]             ${adcHold}                      [get_ports adc*DDR*]
set_input_delay -add_delay -min -clock [get_clocks adcDClkHiDC] -clock_fall ${adcHold}                      [get_ports adc*DDR*]
set_input_delay -add_delay -min -clock [get_clocks adcDClk    ]             ${adcHold}                      [get_ports adc*DDR*]
set_input_delay -add_delay -min -clock [get_clocks adcDClk    ] -clock_fall ${adcHold}                      [get_ports adc*DDR*]

set_false_path -setup -rise_from [get_clocks adcDClkLoDC] -rise_to [get_clocks adcDClkLoDC]
set_false_path -setup -fall_from [get_clocks adcDClkLoDC] -fall_to [get_clocks adcDClkLoDC]
set_false_path -hold  -rise_from [get_clocks adcDClkLoDC] -fall_to [get_clocks adcDClkLoDC]
set_false_path -hold  -fall_from [get_clocks adcDClkLoDC] -rise_to [get_clocks adcDClkLoDC]

set_false_path -setup -rise_from [get_clocks adcDClkHiDC] -rise_to [get_clocks adcDClkHiDC]
set_false_path -setup -fall_from [get_clocks adcDClkHiDC] -fall_to [get_clocks adcDClkHiDC]
set_false_path -hold  -rise_from [get_clocks adcDClkHiDC] -fall_to [get_clocks adcDClkHiDC]
set_false_path -hold  -fall_from [get_clocks adcDClkHiDC] -rise_to [get_clocks adcDClkHiDC]

set_false_path -setup -rise_from [get_clocks adcDClk] -rise_to [get_clocks adcDClk]
set_false_path -setup -fall_from [get_clocks adcDClk] -fall_to [get_clocks adcDClk]
set_false_path -hold  -rise_from [get_clocks adcDClk] -fall_to [get_clocks adcDClk]
set_false_path -hold  -fall_from [get_clocks adcDClk] -rise_to [get_clocks adcDClk]
