EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:+1V8 #PWR?
U 1 1 60B1BF99
P 5900 1950
AR Path="/6086F2E3/60B1BF99" Ref="#PWR?"  Part="1" 
AR Path="/60AABC24/60B1BF99" Ref="#PWR?"  Part="1" 
AR Path="/60B16D71/60B1BF99" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5900 1800 50  0001 C CNN
F 1 "+1V8" H 5915 2123 50  0000 C CNN
F 2 "" H 5900 1950 50  0001 C CNN
F 3 "" H 5900 1950 50  0001 C CNN
	1    5900 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60B1BFA5
P 5600 4550
AR Path="/6086F2E3/60B1BFA5" Ref="#PWR?"  Part="1" 
AR Path="/60AABC24/60B1BFA5" Ref="#PWR?"  Part="1" 
AR Path="/60B16D71/60B1BFA5" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5600 4300 50  0001 C CNN
F 1 "GND" H 5605 4377 50  0000 C CNN
F 2 "" H 5600 4550 50  0001 C CNN
F 3 "" H 5600 4550 50  0001 C CNN
	1    5600 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 4500 5600 4550
Wire Wire Line
	5900 2200 5900 2000
NoConn ~ 6100 4100
NoConn ~ 5100 3900
NoConn ~ 5100 3400
NoConn ~ 6100 3200
NoConn ~ 5100 3100
NoConn ~ 5100 3200
$Comp
L Device:Crystal_GND24_Small Y1
U 1 1 60B1F2AF
P 4700 2800
F 0 "Y1" V 4654 2944 50  0000 L CNN
F 1 "25MHz" V 4700 2350 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_Abracon_ABM10-4Pin_2.5x2.0mm" H 4700 2800 50  0001 C CNN
F 3 "~" H 4700 2800 50  0001 C CNN
F 4 "ABM10W-25.0000MHZ-8-B1U-T3" V 4700 2800 50  0001 C CNN "Part"
F 5 "Abracon" V 4700 2800 50  0001 C CNN "Manufacturer"
	1    4700 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	4700 2700 5100 2700
Wire Wire Line
	4700 2900 5100 2900
Wire Wire Line
	4800 2800 4850 2800
Wire Wire Line
	4850 2800 4850 3000
Wire Wire Line
	4850 3000 4700 3000
Wire Wire Line
	4550 3000 4550 2800
Wire Wire Line
	4550 2800 4600 2800
$Comp
L power:GND #PWR?
U 1 1 60B2062B
P 4700 3000
AR Path="/6086F2E3/60B2062B" Ref="#PWR?"  Part="1" 
AR Path="/60AABC24/60B2062B" Ref="#PWR?"  Part="1" 
AR Path="/60B16D71/60B2062B" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4700 2750 50  0001 C CNN
F 1 "GND" H 4705 2827 50  0000 C CNN
F 2 "" H 4700 3000 50  0001 C CNN
F 3 "" H 4700 3000 50  0001 C CNN
	1    4700 3000
	1    0    0    -1  
$EndComp
Connection ~ 4700 3000
Wire Wire Line
	4700 3000 4550 3000
Text GLabel 4500 3700 0    50   BiDi ~ 0
I2C_SCL
Text GLabel 4500 3600 0    50   BiDi ~ 0
I2C_SDA
Wire Wire Line
	4500 3600 4800 3600
Wire Wire Line
	4500 3700 4650 3700
Text GLabel 6950 2900 2    50   Output ~ 0
FPGA_CLK
Wire Wire Line
	6100 2900 6350 2900
$Comp
L Device:R_Small R?
U 1 1 60B5DFDD
P 4650 3450
AR Path="/60AABC24/60B5DFDD" Ref="R?"  Part="1" 
AR Path="/60B16D71/60B5DFDD" Ref="R23"  Part="1" 
F 0 "R23" V 4900 3450 50  0000 C CNN
F 1 "10k" V 4750 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4650 3450 50  0001 C CNN
F 3 "~" H 4650 3450 50  0001 C CNN
F 4 "RT0603FRE0710KL" V 4650 3450 50  0001 C CNN "Part"
F 5 "Yageo" V 4650 3450 50  0001 C CNN "Manufacturer"
	1    4650 3450
	-1   0    0    1   
$EndComp
Wire Wire Line
	4650 3550 4650 3700
Connection ~ 4650 3700
Wire Wire Line
	4650 3700 5100 3700
Wire Wire Line
	4800 3550 4800 3600
Connection ~ 4800 3600
Wire Wire Line
	4800 3600 5100 3600
$Comp
L power:+1V8 #PWR?
U 1 1 60B5EDFB
P 4250 3300
AR Path="/6086F2E3/60B5EDFB" Ref="#PWR?"  Part="1" 
AR Path="/60AABC24/60B5EDFB" Ref="#PWR?"  Part="1" 
AR Path="/60B16D71/60B5EDFB" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4250 3150 50  0001 C CNN
F 1 "+1V8" H 4265 3473 50  0000 C CNN
F 2 "" H 4250 3300 50  0001 C CNN
F 3 "" H 4250 3300 50  0001 C CNN
	1    4250 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 3300 4250 3350
Wire Wire Line
	4250 3350 4650 3350
Connection ~ 4650 3350
Wire Wire Line
	4650 3350 4800 3350
$Comp
L Device:R_Small R?
U 1 1 60D6AA6F
P 6450 2900
AR Path="/60AABC24/60D6AA6F" Ref="R?"  Part="1" 
AR Path="/60B16D71/60D6AA6F" Ref="R26"  Part="1" 
F 0 "R26" V 6350 2900 50  0000 C CNN
F 1 "33" V 6550 2900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6450 2900 50  0001 C CNN
F 3 "~" H 6450 2900 50  0001 C CNN
F 4 " RC0603FR-0733RL " V 6450 2900 50  0001 C CNN "Part"
F 5 "Yageo" V 6450 2900 50  0001 C CNN "Manufacturer"
	1    6450 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 2900 6950 2900
$Comp
L fmc:100nF_603 C?
U 1 1 60D6D75B
P 4350 2200
AR Path="/6086F2E3/60D6D75B" Ref="C?"  Part="1" 
AR Path="/60B16D71/60D6D75B" Ref="C18"  Part="1" 
F 0 "C18" V 4098 2200 50  0000 C CNN
F 1 "100n" V 4500 2200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4388 2050 50  0001 C CNN
F 3 "~" H 4350 2200 50  0001 C CNN
F 4 "GCJ188R71E104KA12D" H 4475 2400 50  0001 C CNN "Part"
F 5 "Murata" H 4575 2500 50  0001 C CNN "Manufacturer"
	1    4350 2200
	-1   0    0    1   
$EndComp
$Comp
L fmc:1uF_805 C19
U 1 1 60D6E041
P 4750 2200
F 0 "C19" H 4865 2246 50  0000 L CNN
F 1 "1u" H 4865 2155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4788 2050 50  0001 C CNN
F 3 "~" H 4750 2200 50  0001 C CNN
F 4 "GCJ219R71C105KA01" H 4875 2400 50  0001 C CNN "Part"
F 5 "Murata" H 4975 2500 50  0001 C CNN "Manufacturer"
	1    4750 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB2
U 1 1 60D6EF94
P 5500 1950
F 0 "FB2" V 5263 1950 50  0000 C CNN
F 1 "Ferrite_Bead" V 5354 1950 50  0000 C CNN
F 2 "Inductor_SMD:L_0603_1608Metric" V 5430 1950 50  0001 C CNN
F 3 "~" H 5500 1950 50  0001 C CNN
F 4 "MPZ1608S221ATA00" H 5500 1950 50  0001 C CNN "Part"
F 5 "TDK" H 5500 1950 50  0001 C CNN "Manufacturer"
	1    5500 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	4350 2050 4750 2050
Wire Wire Line
	4750 2050 5000 2050
Wire Wire Line
	5300 2050 5300 2200
Connection ~ 4750 2050
Wire Wire Line
	5300 2050 5300 1950
Wire Wire Line
	5300 1950 5400 1950
Connection ~ 5300 2050
Wire Wire Line
	5600 1950 5900 1950
Connection ~ 5900 1950
Wire Wire Line
	4750 2350 4600 2350
$Comp
L power:GND #PWR?
U 1 1 60D72118
P 4600 2350
AR Path="/6086F2E3/60D72118" Ref="#PWR?"  Part="1" 
AR Path="/60AABC24/60D72118" Ref="#PWR?"  Part="1" 
AR Path="/60B16D71/60D72118" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 4600 2100 50  0001 C CNN
F 1 "GND" H 4605 2177 50  0000 C CNN
F 2 "" H 4600 2350 50  0001 C CNN
F 3 "" H 4600 2350 50  0001 C CNN
	1    4600 2350
	1    0    0    -1  
$EndComp
Connection ~ 4600 2350
Wire Wire Line
	4600 2350 4350 2350
$Comp
L power:PWR_FLAG #FLG?
U 1 1 60D72FF6
P 5000 2050
F 0 "#FLG?" H 5000 2125 50  0001 C CNN
F 1 "PWR_FLAG" H 5000 2223 50  0000 C CNN
F 2 "" H 5000 2050 50  0001 C CNN
F 3 "~" H 5000 2050 50  0001 C CNN
	1    5000 2050
	1    0    0    -1  
$EndComp
Connection ~ 5000 2050
Wire Wire Line
	5000 2050 5300 2050
Wire Wire Line
	6100 3500 6350 3500
Wire Wire Line
	6100 3800 6350 3800
$Comp
L Connector:Conn_Coaxial J8
U 1 1 6092C0EF
P 7300 3800
F 0 "J8" H 7400 3775 50  0000 L CNN
F 1 "BNC" H 7400 3684 50  0000 L CNN
F 2 "proj_footprints:BNC_Molex_0731713150" H 7300 3800 50  0001 C CNN
F 3 " ~" H 7300 3800 50  0001 C CNN
F 4 "0731713150" H 7300 3800 50  0001 C CNN "Part"
F 5 "Molex" H 7300 3800 50  0001 C CNN "Manufacturer"
	1    7300 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 3800 7100 3800
Wire Wire Line
	7300 3700 7050 3700
Wire Wire Line
	7050 3700 7050 4000
Wire Wire Line
	7050 4000 7300 4000
$Comp
L power:GND #PWR?
U 1 1 6092DB98
P 7050 4000
AR Path="/6086F2E3/6092DB98" Ref="#PWR?"  Part="1" 
AR Path="/60AABC24/6092DB98" Ref="#PWR?"  Part="1" 
AR Path="/60B16D71/6092DB98" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7050 3750 50  0001 C CNN
F 1 "GND" H 7055 3827 50  0000 C CNN
F 2 "" H 7050 4000 50  0001 C CNN
F 3 "" H 7050 4000 50  0001 C CNN
	1    7050 4000
	1    0    0    -1  
$EndComp
Connection ~ 7050 4000
$Comp
L fmc:100nF_603 C?
U 1 1 60B563A5
P 6500 2150
AR Path="/6086F2E3/60B563A5" Ref="C?"  Part="1" 
AR Path="/60B16D71/60B563A5" Ref="C40"  Part="1" 
F 0 "C40" H 6500 1850 50  0000 C CNN
F 1 "100n" H 6500 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6538 2000 50  0001 C CNN
F 3 "~" H 6500 2150 50  0001 C CNN
F 4 "GCJ188R71E104KA12D" H 6625 2350 50  0001 C CNN "Part"
F 5 "Murata" H 6725 2450 50  0001 C CNN "Manufacturer"
	1    6500 2150
	-1   0    0    1   
$EndComp
$Comp
L fmc:100nF_603 C?
U 1 1 60B58071
P 6700 2150
AR Path="/6086F2E3/60B58071" Ref="C?"  Part="1" 
AR Path="/60B16D71/60B58071" Ref="C41"  Part="1" 
F 0 "C41" H 6700 1850 50  0000 C CNN
F 1 "100n" H 6700 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6738 2000 50  0001 C CNN
F 3 "~" H 6700 2150 50  0001 C CNN
F 4 "GCJ188R71E104KA12D" H 6825 2350 50  0001 C CNN "Part"
F 5 "Murata" H 6925 2450 50  0001 C CNN "Manufacturer"
	1    6700 2150
	-1   0    0    1   
$EndComp
$Comp
L fmc:100nF_603 C?
U 1 1 60B58488
P 6900 2150
AR Path="/6086F2E3/60B58488" Ref="C?"  Part="1" 
AR Path="/60B16D71/60B58488" Ref="C42"  Part="1" 
F 0 "C42" H 6900 1850 50  0000 C CNN
F 1 "100n" H 6900 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6938 2000 50  0001 C CNN
F 3 "~" H 6900 2150 50  0001 C CNN
F 4 "GCJ188R71E104KA12D" H 7025 2350 50  0001 C CNN "Part"
F 5 "Murata" H 7125 2450 50  0001 C CNN "Manufacturer"
	1    6900 2150
	-1   0    0    1   
$EndComp
$Comp
L fmc:100nF_603 C?
U 1 1 60B58802
P 7100 2150
AR Path="/6086F2E3/60B58802" Ref="C?"  Part="1" 
AR Path="/60B16D71/60B58802" Ref="C43"  Part="1" 
F 0 "C43" H 7100 1850 50  0000 C CNN
F 1 "100n" H 7100 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7138 2000 50  0001 C CNN
F 3 "~" H 7100 2150 50  0001 C CNN
F 4 "GCJ188R71E104KA12D" H 7225 2350 50  0001 C CNN "Part"
F 5 "Murata" H 7325 2450 50  0001 C CNN "Manufacturer"
	1    7100 2150
	-1   0    0    1   
$EndComp
$Comp
L fmc:100nF_603 C?
U 1 1 60B58E63
P 7300 2150
AR Path="/6086F2E3/60B58E63" Ref="C?"  Part="1" 
AR Path="/60B16D71/60B58E63" Ref="C44"  Part="1" 
F 0 "C44" H 7300 1850 50  0000 C CNN
F 1 "100n" H 7300 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7338 2000 50  0001 C CNN
F 3 "~" H 7300 2150 50  0001 C CNN
F 4 "GCJ188R71E104KA12D" H 7425 2350 50  0001 C CNN "Part"
F 5 "Murata" H 7525 2450 50  0001 C CNN "Manufacturer"
	1    7300 2150
	-1   0    0    1   
$EndComp
$Comp
L fmc:100nF_603 C?
U 1 1 60B59237
P 7500 2150
AR Path="/6086F2E3/60B59237" Ref="C?"  Part="1" 
AR Path="/60B16D71/60B59237" Ref="C45"  Part="1" 
F 0 "C45" H 7500 1850 50  0000 C CNN
F 1 "100n" H 7500 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7538 2000 50  0001 C CNN
F 3 "~" H 7500 2150 50  0001 C CNN
F 4 "GCJ188R71E104KA12D" H 7625 2350 50  0001 C CNN "Part"
F 5 "Murata" H 7725 2450 50  0001 C CNN "Manufacturer"
	1    7500 2150
	-1   0    0    1   
$EndComp
$Comp
L fmc:1uF_805 C46
U 1 1 60B5951C
P 7800 2150
F 0 "C46" H 7750 2450 50  0000 L CNN
F 1 "1u" H 7750 1850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 7838 2000 50  0001 C CNN
F 3 "~" H 7800 2150 50  0001 C CNN
F 4 "GCJ219R71C105KA01" H 7925 2350 50  0001 C CNN "Part"
F 5 "Murata" H 8025 2450 50  0001 C CNN "Manufacturer"
	1    7800 2150
	1    0    0    -1  
$EndComp
$Comp
L fmc:1uF_805 C47
U 1 1 60B59F17
P 8050 2150
F 0 "C47" H 8000 2450 50  0000 L CNN
F 1 "1u" H 8000 1850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8088 2000 50  0001 C CNN
F 3 "~" H 8050 2150 50  0001 C CNN
F 4 "GCJ219R71C105KA01" H 8175 2350 50  0001 C CNN "Part"
F 5 "Murata" H 8275 2450 50  0001 C CNN "Manufacturer"
	1    8050 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 2000 7800 2000
Connection ~ 5900 2000
Wire Wire Line
	5900 2000 5900 1950
Connection ~ 6500 2000
Wire Wire Line
	6500 2000 5900 2000
Connection ~ 6700 2000
Wire Wire Line
	6700 2000 6500 2000
Connection ~ 6900 2000
Wire Wire Line
	6900 2000 6700 2000
Connection ~ 7100 2000
Wire Wire Line
	7100 2000 6900 2000
Connection ~ 7300 2000
Wire Wire Line
	7300 2000 7100 2000
Connection ~ 7500 2000
Wire Wire Line
	7500 2000 7300 2000
Connection ~ 7800 2000
Wire Wire Line
	7800 2000 7500 2000
Wire Wire Line
	6500 2300 6700 2300
Connection ~ 6700 2300
Wire Wire Line
	6700 2300 6900 2300
Connection ~ 6900 2300
Wire Wire Line
	6900 2300 7100 2300
Connection ~ 7100 2300
Wire Wire Line
	7100 2300 7300 2300
Connection ~ 7300 2300
Wire Wire Line
	7300 2300 7500 2300
Connection ~ 7500 2300
Wire Wire Line
	7500 2300 7800 2300
Connection ~ 7800 2300
Wire Wire Line
	7800 2300 8050 2300
$Comp
L power:GND #PWR?
U 1 1 60B5BE4D
P 6250 2300
AR Path="/6086F2E3/60B5BE4D" Ref="#PWR?"  Part="1" 
AR Path="/60AABC24/60B5BE4D" Ref="#PWR?"  Part="1" 
AR Path="/60B16D71/60B5BE4D" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6250 2050 50  0001 C CNN
F 1 "GND" H 6255 2127 50  0000 C CNN
F 2 "" H 6250 2300 50  0001 C CNN
F 3 "" H 6250 2300 50  0001 C CNN
	1    6250 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 2300 6250 2300
Connection ~ 6500 2300
$Comp
L Device:C_Small C?
U 1 1 60B659D8
P 8400 2150
AR Path="/6086F2E3/60B659D8" Ref="C?"  Part="1" 
AR Path="/60B16D71/60B659D8" Ref="C48"  Part="1" 
F 0 "C48" H 8350 2450 50  0000 L CNN
F 1 "10u" H 8350 1850 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8400 2150 50  0001 C CNN
F 3 "~" H 8400 2150 50  0001 C CNN
F 4 "CNC5L1X7R1C106K160AE" H 8400 2150 50  0001 C CNN "Part"
F 5 "TDK" H 8400 2150 50  0001 C CNN "Manufacturer"
	1    8400 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 2000 8400 2000
Wire Wire Line
	8400 2000 8400 2050
Connection ~ 8050 2000
Wire Wire Line
	8050 2300 8400 2300
Wire Wire Line
	8400 2300 8400 2250
Connection ~ 8050 2300
$Comp
L Device:R_Small R?
U 1 1 60C3571E
P 4800 3450
AR Path="/60AABC24/60C3571E" Ref="R?"  Part="1" 
AR Path="/60B16D71/60C3571E" Ref="R24"  Part="1" 
F 0 "R24" V 5000 3450 50  0000 C CNN
F 1 "10k" V 4900 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4800 3450 50  0001 C CNN
F 3 "~" H 4800 3450 50  0001 C CNN
F 4 "RT0603FRE0710KL" V 4800 3450 50  0001 C CNN "Part"
F 5 "Yageo" V 4800 3450 50  0001 C CNN "Manufacturer"
	1    4800 3450
	1    0    0    1   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60C371EA
P 6450 3500
AR Path="/60AABC24/60C371EA" Ref="R?"  Part="1" 
AR Path="/60B16D71/60C371EA" Ref="R27"  Part="1" 
F 0 "R27" V 6350 3500 50  0000 C CNN
F 1 "33" V 6550 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6450 3500 50  0001 C CNN
F 3 "~" H 6450 3500 50  0001 C CNN
F 4 " RC0603FR-0733RL " V 6450 3500 50  0001 C CNN "Part"
F 5 "Yageo" V 6450 3500 50  0001 C CNN "Manufacturer"
	1    6450 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 60C375F6
P 6450 3800
AR Path="/60AABC24/60C375F6" Ref="R?"  Part="1" 
AR Path="/60B16D71/60C375F6" Ref="R28"  Part="1" 
F 0 "R28" V 6350 3800 50  0000 C CNN
F 1 "33" V 6550 3800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6450 3800 50  0001 C CNN
F 3 "~" H 6450 3800 50  0001 C CNN
F 4 " RC0603FR-0733RL " V 6450 3800 50  0001 C CNN "Part"
F 5 "Yageo" V 6450 3800 50  0001 C CNN "Manufacturer"
	1    6450 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 2200 5500 2200
$Comp
L fmc:5P49V5925 U7
U 1 1 60C70124
P 5600 3400
F 0 "U7" H 5900 2300 50  0000 C CNN
F 1 "5P49V5925" H 6050 2150 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.8x2.8mm" H 5550 2300 50  0001 C CNN
F 3 "https://www.idt.com/document/dst/5p49v5925-datasheet" H 5150 4550 50  0001 C CNN
F 4 "5P49V5925" H 5600 2029 50  0001 C CNN "Part"
F 5 "Renesas" H 5600 2029 50  0001 C CNN "Manufacturer"
	1    5600 3400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial J7
U 1 1 60C731B4
P 7300 3500
F 0 "J7" H 7400 3475 50  0000 L CNN
F 1 "BNC" H 7400 3384 50  0000 L CNN
F 2 "proj_footprints:BNC_Molex_0731713150" H 7300 3500 50  0001 C CNN
F 3 " ~" H 7300 3500 50  0001 C CNN
F 4 "0731713150" H 7300 3500 50  0001 C CNN "Part"
F 5 "Molex" H 7300 3500 50  0001 C CNN "Manufacturer"
	1    7300 3500
	1    0    0    -1  
$EndComp
$Comp
L fmc:100nF_603 C?
U 1 1 60B85B4A
P 6800 3500
AR Path="/6086F2E3/60B85B4A" Ref="C?"  Part="1" 
AR Path="/60B16D71/60B85B4A" Ref="C49"  Part="1" 
F 0 "C49" V 6950 3500 50  0000 C CNN
F 1 "100n" V 6650 3500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6838 3350 50  0001 C CNN
F 3 "~" H 6800 3500 50  0001 C CNN
F 4 "GCJ188R71E104KA12D" H 6925 3700 50  0001 C CNN "Part"
F 5 "Murata" H 7025 3800 50  0001 C CNN "Manufacturer"
	1    6800 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6950 3500 7100 3500
Wire Wire Line
	6550 3500 6650 3500
Wire Wire Line
	5500 2200 5600 2200
Connection ~ 5500 2200
Connection ~ 5900 2200
Connection ~ 5600 2200
Wire Wire Line
	5600 2200 5700 2200
Connection ~ 5700 2200
Wire Wire Line
	5700 2200 5800 2200
Connection ~ 5800 2200
Wire Wire Line
	5800 2200 5900 2200
$EndSCHEMATC
