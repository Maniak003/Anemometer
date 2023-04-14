EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L Amp200-rescue:MAX4477-My_Library D1
U 1 1 64214348
P 4200 3000
F 0 "D1" H 4000 3100 50  0000 L CNN
F 1 "MAX4477" H 4000 3000 50  0000 L CNN
F 2 "Package_SO:MSOP-8_3x3mm_P0.65mm" H 4200 3000 50  0001 C CNN
F 3 "" H 4200 3000 50  0001 C CNN
	1    4200 3000
	1    0    0    -1  
$EndComp
$Comp
L Amp200-rescue:MAX4477-My_Library D1
U 2 1 64214619
P 6300 3650
F 0 "D1" H 6200 3650 50  0000 C CNN
F 1 "MAX4477" H 6300 3974 50  0001 C CNN
F 2 "Package_SO:MSOP-8_3x3mm_P0.65mm" H 6300 3650 50  0001 C CNN
F 3 "" H 6300 3650 50  0001 C CNN
	2    6300 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 64214ACC
P 5650 4050
F 0 "C3" H 5400 4100 50  0000 L CNN
F 1 "1u" H 5400 4000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5688 3900 50  0001 C CNN
F 3 "~" H 5650 4050 50  0001 C CNN
	1    5650 4050
	1    0    0    -1  
$EndComp
$Comp
L Amp200-rescue:Conn-My_Library J7
U 1 1 64215621
P 8500 4500
F 0 "J7" H 8528 4546 50  0001 L CNN
F 1 "Conn" H 8528 4455 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 8550 4350 50  0001 C CNN
F 3 "~" H 8500 4500 50  0001 C CNN
	1    8500 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 64215CB0
P 8100 4250
F 0 "#PWR06" H 8100 4000 50  0001 C CNN
F 1 "GND" H 8105 4077 50  0001 C CNN
F 2 "" H 8100 4250 50  0001 C CNN
F 3 "" H 8100 4250 50  0001 C CNN
	1    8100 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6421CB29
P 4450 2650
F 0 "R2" V 4550 2650 50  0000 C CNN
F 1 "3M" V 4650 2650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4380 2650 50  0001 C CNN
F 3 "~" H 4450 2650 50  0001 C CNN
	1    4450 2650
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 6421D1E2
P 3700 2850
F 0 "R1" V 3493 2850 50  0000 C CNN
F 1 "10k" V 3584 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3630 2850 50  0001 C CNN
F 3 "~" H 3700 2850 50  0001 C CNN
	1    3700 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 6421FE15
P 5300 3300
F 0 "R3" V 5507 3300 50  0000 C CNN
F 1 "10k" V 5416 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5230 3300 50  0001 C CNN
F 3 "~" H 5300 3300 50  0001 C CNN
	1    5300 3300
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 642222EA
P 5600 3300
F 0 "C2" V 5750 3300 50  0000 C CNN
F 1 "100n" V 5850 3300 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5638 3150 50  0001 C CNN
F 3 "~" H 5600 3300 50  0001 C CNN
	1    5600 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	3850 3800 3850 3150
Wire Wire Line
	3850 2850 3850 2650
Wire Wire Line
	3850 2650 4300 2650
$Comp
L Amp200-rescue:TLV3501-2-My_Library D3
U 1 1 64231BA0
P 7750 3750
F 0 "D3" H 7800 3900 50  0000 L CNN
F 1 "TLV3501" H 7550 3750 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 7850 3750 50  0001 C CNN
F 3 "" H 7850 3750 50  0001 C CNN
	1    7750 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 64232394
P 6800 3650
F 0 "C8" V 6900 3500 50  0000 C CNN
F 1 "1.5n" V 7000 3500 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6838 3500 50  0001 C CNN
F 3 "~" H 6800 3650 50  0001 C CNN
	1    6800 3650
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 64232CAC
P 7350 4050
F 0 "R9" H 7420 4096 50  0000 L CNN
F 1 "5.1k" H 7420 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7280 4050 50  0001 C CNN
F 3 "~" H 7350 4050 50  0001 C CNN
	1    7350 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 64233143
P 7150 4050
F 0 "C10" H 7000 4300 50  0000 L CNN
F 1 "1u" H 7000 4200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7188 3900 50  0001 C CNN
F 3 "~" H 7150 4050 50  0001 C CNN
	1    7150 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 4250 7750 3950
$Comp
L Device:R R8
U 1 1 64233EF3
P 7350 2900
F 0 "R8" H 7150 2850 50  0000 L CNN
F 1 "100k" H 7100 2950 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7280 2900 50  0001 C CNN
F 3 "~" H 7350 2900 50  0001 C CNN
	1    7350 2900
	-1   0    0    1   
$EndComp
Wire Wire Line
	7450 3850 7350 3850
Connection ~ 7350 3850
Wire Wire Line
	7350 3850 7150 3850
$Comp
L Device:R R7
U 1 1 642413F6
P 6950 4050
F 0 "R7" H 6750 4100 50  0000 L CNN
F 1 "10k" H 6750 4000 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6880 4050 50  0001 C CNN
F 3 "~" H 6950 4050 50  0001 C CNN
	1    6950 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 3800 5850 3800
$Comp
L Device:R R4
U 1 1 6425AE1D
P 5850 2900
F 0 "R4" H 5600 2950 50  0000 L CNN
F 1 "100k" H 5550 2850 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5780 2900 50  0001 C CNN
F 3 "~" H 5850 2900 50  0001 C CNN
	1    5850 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 3250 4150 4250
Connection ~ 7750 4250
$Comp
L Device:C C1
U 1 1 64268B9E
P 2800 2850
F 0 "C1" V 2548 2850 50  0000 C CNN
F 1 "1.5n" V 2639 2850 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2838 2700 50  0001 C CNN
F 3 "~" H 2800 2850 50  0001 C CNN
	1    2800 2850
	0    1    1    0   
$EndComp
$Comp
L Amp200-rescue:Conn-My_Library J6
U 1 1 6426C541
P 8500 3750
F 0 "J6" H 8528 3796 50  0001 L CNN
F 1 "Conn" H 8528 3705 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 8550 3600 50  0001 C CNN
F 3 "~" H 8500 3750 50  0001 C CNN
	1    8500 3750
	1    0    0    -1  
$EndComp
$Comp
L Amp200-rescue:Conn-My_Library J8
U 1 1 6426CAB6
P 8500 4250
F 0 "J8" H 8528 4296 50  0001 L CNN
F 1 "Conn" H 8528 4205 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 8550 4100 50  0001 C CNN
F 3 "~" H 8500 4250 50  0001 C CNN
	1    8500 4250
	1    0    0    -1  
$EndComp
$Comp
L Amp200-rescue:Conn-My_Library J9
U 1 1 6426D093
P 7350 1750
F 0 "J9" H 7378 1796 50  0001 L CNN
F 1 "Conn" H 7378 1705 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 7400 1600 50  0001 C CNN
F 3 "~" H 7350 1750 50  0001 C CNN
	1    7350 1750
	1    0    0    -1  
$EndComp
$Comp
L Amp200-rescue:Conn-My_Library J2
U 1 1 6426D58E
P 4450 3200
F 0 "J2" H 4478 3246 50  0001 L CNN
F 1 "Conn" H 4478 3155 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 4500 3050 50  0001 C CNN
F 3 "~" H 4450 3200 50  0001 C CNN
	1    4450 3200
	-1   0    0    1   
$EndComp
$Comp
L Amp200-rescue:Conn-My_Library J3
U 1 1 6426DCFC
P 4450 3300
F 0 "J3" H 4478 3346 50  0001 L CNN
F 1 "Conn" H 4478 3255 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 4500 3150 50  0001 C CNN
F 3 "~" H 4450 3300 50  0001 C CNN
	1    4450 3300
	-1   0    0    1   
$EndComp
$Comp
L Amp200-rescue:Conn-My_Library J1
U 1 1 6426E9F0
P 2450 2850
F 0 "J1" H 2478 2896 50  0001 L CNN
F 1 "Conn" H 2478 2805 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 2500 2700 50  0001 C CNN
F 3 "~" H 2450 2850 50  0001 C CNN
	1    2450 2850
	-1   0    0    1   
$EndComp
Connection ~ 6650 3650
$Comp
L Device:R R5
U 1 1 64225E4E
P 5850 4050
F 0 "R5" H 5920 4096 50  0000 L CNN
F 1 "100k" H 5920 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5780 4050 50  0001 C CNN
F 3 "~" H 5850 4050 50  0001 C CNN
	1    5850 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2650 4700 2650
Wire Wire Line
	4700 2650 4700 3000
Wire Wire Line
	4700 3000 4550 3000
Connection ~ 4700 2650
Wire Wire Line
	4700 2650 5150 2650
Wire Wire Line
	6950 3650 7450 3650
Connection ~ 3850 2850
$Comp
L Device:C C6
U 1 1 642C0DCF
P 6950 2650
F 0 "C6" H 6900 2350 50  0000 L CNN
F 1 ".1u" H 6900 2250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6988 2500 50  0001 C CNN
F 3 "~" H 6950 2650 50  0001 C CNN
	1    6950 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 642C1288
P 6750 2650
F 0 "C5" H 6700 2350 50  0000 L CNN
F 1 ".1u" H 6700 2250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6788 2500 50  0001 C CNN
F 3 "~" H 6750 2650 50  0001 C CNN
	1    6750 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 642C291A
P 6550 2650
F 0 "C4" H 6500 2350 50  0000 L CNN
F 1 ".1u" H 6500 2250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6588 2500 50  0001 C CNN
F 3 "~" H 6550 2650 50  0001 C CNN
	1    6550 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 642C392F
P 6950 2800
F 0 "#PWR03" H 6950 2550 50  0001 C CNN
F 1 "GND" H 6955 2627 50  0001 C CNN
F 2 "" H 6950 2800 50  0001 C CNN
F 3 "" H 6950 2800 50  0001 C CNN
	1    6950 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 642C3DF1
P 6750 2800
F 0 "#PWR02" H 6750 2550 50  0001 C CNN
F 1 "GND" H 6755 2627 50  0001 C CNN
F 2 "" H 6750 2800 50  0001 C CNN
F 3 "" H 6750 2800 50  0001 C CNN
	1    6750 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 642C42CA
P 6550 2800
F 0 "#PWR01" H 6550 2550 50  0001 C CNN
F 1 "GND" H 6555 2627 50  0001 C CNN
F 2 "" H 6550 2800 50  0001 C CNN
F 3 "" H 6550 2800 50  0001 C CNN
	1    6550 2800
	1    0    0    -1  
$EndComp
$Comp
L Amp200-rescue:Conn-My_Library J5
U 1 1 642CF5AB
P 8500 3300
F 0 "J5" H 8528 3346 50  0001 L CNN
F 1 "Conn" H 8528 3255 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 8550 3150 50  0001 C CNN
F 3 "~" H 8500 3300 50  0001 C CNN
	1    8500 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 3150 3250 4250
Connection ~ 4150 4250
Wire Wire Line
	7850 3950 7850 4500
Wire Wire Line
	7850 4500 3550 4500
$Comp
L Device:C C13
U 1 1 64367121
P 5450 2650
F 0 "C13" H 5350 2350 50  0000 L CNN
F 1 ".1u" H 5350 2250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5488 2500 50  0001 C CNN
F 3 "~" H 5450 2650 50  0001 C CNN
	1    5450 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 64367880
P 5450 2800
F 0 "#PWR08" H 5450 2550 50  0001 C CNN
F 1 "GND" H 5455 2627 50  0001 C CNN
F 2 "" H 5450 2800 50  0001 C CNN
F 3 "" H 5450 2800 50  0001 C CNN
	1    5450 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 6436D251
P 7000 3300
F 0 "R10" V 7100 3300 50  0000 C CNN
F 1 "50" V 7200 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6930 3300 50  0001 C CNN
F 3 "~" H 7000 3300 50  0001 C CNN
	1    7000 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	7150 3300 8400 3300
Wire Wire Line
	8050 3750 8400 3750
$Comp
L Device:C C14
U 1 1 64392278
P 8100 4050
F 0 "C14" H 8250 4100 50  0000 L CNN
F 1 "1u" H 8250 4000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8138 3900 50  0001 C CNN
F 3 "~" H 8100 4050 50  0001 C CNN
	1    8100 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 4500 8400 4500
Connection ~ 7850 4500
Wire Wire Line
	6950 3900 6950 3650
Connection ~ 6950 3650
Wire Wire Line
	6950 4200 6950 4250
Connection ~ 6950 4250
Wire Wire Line
	6950 4250 7150 4250
Wire Wire Line
	7150 4200 7150 4250
Connection ~ 7150 4250
Wire Wire Line
	7150 4250 7350 4250
Wire Wire Line
	7350 4200 7350 4250
Connection ~ 7350 4250
Wire Wire Line
	7350 4250 7750 4250
Wire Wire Line
	7350 3850 7350 3900
Wire Wire Line
	7150 3850 7150 3900
Wire Wire Line
	7750 4250 8100 4250
Wire Wire Line
	8100 4200 8100 4250
Connection ~ 8100 4250
Wire Wire Line
	8100 4250 8400 4250
$Comp
L Amp200-rescue:TS5A4595-My_Library D5
U 1 1 643F68CC
P 3250 3050
F 0 "D5" H 3300 3100 50  0000 L CNN
F 1 "TS5A4595" H 3300 2950 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 3250 3050 50  0001 C CNN
F 3 "" H 3250 3050 50  0001 C CNN
	1    3250 3050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3250 4250 4150 4250
Wire Wire Line
	5850 4200 5850 4250
Connection ~ 5850 4250
Wire Wire Line
	5850 4250 6950 4250
Wire Wire Line
	5850 3900 5850 3800
Connection ~ 5850 3800
Wire Wire Line
	5850 3800 5850 3050
Wire Wire Line
	5650 3900 5650 3800
Connection ~ 5650 3800
Wire Wire Line
	5650 4200 5650 4250
Connection ~ 5650 4250
Wire Wire Line
	3850 3800 5650 3800
Wire Wire Line
	5650 3800 5850 3800
Wire Wire Line
	5650 4250 5850 4250
Wire Wire Line
	6450 3300 6650 3300
Wire Wire Line
	4150 4250 4850 4250
Wire Wire Line
	5750 3300 5950 3300
Wire Wire Line
	6650 3300 6650 3650
Connection ~ 6650 3300
Wire Wire Line
	6650 3300 6850 3300
Wire Wire Line
	5950 3300 5950 3500
Connection ~ 5950 3300
Wire Wire Line
	4850 3600 4850 3650
Connection ~ 4850 4250
Wire Wire Line
	4850 4250 5650 4250
Wire Wire Line
	5950 3300 6150 3300
$Comp
L Device:R R6
U 1 1 64214E19
P 6300 3300
F 0 "R6" V 6100 3300 50  0000 C CNN
F 1 "100k" V 6200 3300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6230 3300 50  0001 C CNN
F 3 "~" H 6300 3300 50  0001 C CNN
	1    6300 3300
	0    1    1    0   
$EndComp
Text Label 8050 3300 0    50   ~ 0
AnalogOut
Text Label 7100 1750 0    50   ~ 0
+12v
Text Label 8250 3750 0    50   ~ 0
Zero
Text Label 8200 4500 0    50   ~ 0
Select
Text Label 2550 2850 0    50   ~ 0
Input
Wire Wire Line
	2550 2850 2650 2850
Wire Wire Line
	8100 3450 7750 3450
Connection ~ 7750 3450
Wire Wire Line
	7750 3450 7750 3550
Wire Wire Line
	8100 3450 8100 3900
$Comp
L Device:L L2
U 1 1 6428B384
P 6700 1750
F 0 "L2" V 6890 1750 50  0000 C CNN
F 1 "47u" V 6799 1750 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric" H 6700 1750 50  0001 C CNN
F 3 "~" H 6700 1750 50  0001 C CNN
	1    6700 1750
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C7
U 1 1 6428BEBA
P 6500 2050
F 0 "C7" H 6550 2250 50  0000 L CNN
F 1 "10u" H 6550 2150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6538 1900 50  0001 C CNN
F 3 "~" H 6500 2050 50  0001 C CNN
	1    6500 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 6428C791
P 6900 2050
F 0 "C15" H 6950 2250 50  0000 L CNN
F 1 "10u" H 6950 2150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6938 1900 50  0001 C CNN
F 3 "~" H 6900 2050 50  0001 C CNN
	1    6900 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 64290704
P 6200 2200
F 0 "#PWR04" H 6200 1950 50  0001 C CNN
F 1 "GND" H 6205 2027 50  0001 C CNN
F 2 "" H 6200 2200 50  0001 C CNN
F 3 "" H 6200 2200 50  0001 C CNN
	1    6200 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 1750 6900 1750
Wire Wire Line
	6500 1900 6500 1750
Wire Wire Line
	6900 1900 6900 1750
Connection ~ 6900 1750
Wire Wire Line
	6900 1750 7250 1750
Text Label 7450 2500 0    50   ~ 0
+3.3v
Wire Wire Line
	7350 2750 7350 2500
Wire Wire Line
	7350 3050 7350 3850
Wire Wire Line
	7750 3450 7750 2500
Wire Wire Line
	5850 1750 5900 1750
$Comp
L Device:C C16
U 1 1 64356C37
P 6350 2650
F 0 "C16" H 6250 2350 50  0000 L CNN
F 1 "22u" H 6250 2250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6388 2500 50  0001 C CNN
F 3 "~" H 6350 2650 50  0001 C CNN
	1    6350 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 643574AC
P 6350 2800
F 0 "#PWR09" H 6350 2550 50  0001 C CNN
F 1 "GND" H 6355 2627 50  0001 C CNN
F 2 "" H 6350 2800 50  0001 C CNN
F 3 "" H 6350 2800 50  0001 C CNN
	1    6350 2800
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:NCP1117-3.3_SOT223 U2
U 1 1 64385714
P 6200 1750
F 0 "U2" H 6200 1992 50  0000 C CNN
F 1 "NCP1117-3.3" H 6200 1901 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 6200 1950 50  0001 C CNN
F 3 "http://www.onsemi.com/pub_link/Collateral/NCP1117-D.PDF" H 6300 1500 50  0001 C CNN
	1    6200 1750
	-1   0    0    -1  
$EndComp
Connection ~ 6500 1750
Wire Wire Line
	6500 2200 6200 2200
Wire Wire Line
	6200 2050 6200 2200
Connection ~ 6200 2200
Connection ~ 7350 2500
Wire Wire Line
	6550 2500 6750 2500
Connection ~ 6750 2500
Wire Wire Line
	6750 2500 6950 2500
Connection ~ 6950 2500
Wire Wire Line
	7350 2500 7750 2500
Wire Wire Line
	4150 2500 4850 2500
Wire Wire Line
	4150 2500 4150 2750
Connection ~ 5850 2500
Wire Wire Line
	5850 2500 5850 2750
Wire Wire Line
	5850 2500 6350 2500
Connection ~ 6550 2500
Wire Wire Line
	5850 1750 5850 2500
Wire Wire Line
	6500 1750 6550 1750
Wire Wire Line
	6500 2200 6900 2200
Connection ~ 6500 2200
$Comp
L Amp200-rescue:Conn-My_Library J10
U 1 1 64445C9E
P 2450 4250
F 0 "J10" H 2478 4296 50  0001 L CNN
F 1 "Conn" H 2478 4205 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 2500 4100 50  0001 C CNN
F 3 "~" H 2450 4250 50  0001 C CNN
	1    2450 4250
	-1   0    0    1   
$EndComp
Wire Wire Line
	2550 4250 3250 4250
Connection ~ 3250 4250
Wire Wire Line
	3550 2950 3550 4500
$Comp
L My_Library:AD5245 D2
U 1 1 643A72C4
P 4850 3400
F 0 "D2" H 4950 3800 50  0000 C CNN
F 1 "AD5245" H 5050 3050 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-8" H 4850 3400 50  0001 C CNN
F 3 "" H 4850 3400 50  0001 C CNN
	1    4850 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2650 5150 3200
Wire Wire Line
	5150 3400 5150 3650
Wire Wire Line
	5150 3650 4850 3650
Connection ~ 4850 3650
Wire Wire Line
	4850 3650 4850 4250
Wire Wire Line
	4550 3400 4550 3650
Wire Wire Line
	4550 3650 4850 3650
Wire Wire Line
	4850 3000 4850 2500
Connection ~ 4850 2500
Wire Wire Line
	4850 2500 5450 2500
Connection ~ 5450 2500
Wire Wire Line
	5450 2500 5850 2500
Wire Wire Line
	4150 2500 3250 2500
Wire Wire Line
	3250 2500 3250 2650
Connection ~ 4150 2500
Connection ~ 6350 2500
Wire Wire Line
	6350 2500 6550 2500
Wire Wire Line
	6950 2500 7350 2500
$EndSCHEMATC
