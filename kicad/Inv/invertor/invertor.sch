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
L Diode:BAT54A D3
U 1 1 64528315
P 5050 2750
F 0 "D3" H 5150 2650 50  0000 C CNN
F 1 "BAT54A" H 5200 2550 50  0001 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5125 2875 50  0001 L CNN
F 3 "http://www.diodes.com/_files/datasheets/ds11005.pdf" H 4930 2750 50  0001 C CNN
	1    5050 2750
	1    0    0    -1  
$EndComp
$Comp
L Diode:BAT54S D4
U 1 1 64528AC9
P 5700 2050
F 0 "D4" V 5550 1850 50  0000 L CNN
F 1 "BAT54S" V 5600 1650 50  0001 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5775 2175 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11005.pdf" H 5580 2050 50  0001 C CNN
	1    5700 2050
	0    -1   1    0   
$EndComp
$Comp
L Diode:BAT54C D2
U 1 1 645294D1
P 5050 1550
F 0 "D2" H 5050 1700 50  0000 C CNN
F 1 "BAT54C" H 5050 1800 50  0001 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 5125 1675 50  0001 L CNN
F 3 "http://www.diodes.com/_files/datasheets/ds11005.pdf" H 4970 1550 50  0001 C CNN
	1    5050 1550
	1    0    0    1   
$EndComp
$Comp
L Transistor_FET:IRF7343PBF Q2
U 2 1 6452A028
P 5600 1550
F 0 "Q2" H 5804 1596 50  0000 L CNN
F 1 "IRF7343PBF" H 5804 1505 50  0001 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5800 1475 50  0001 L CNN
F 3 "http://www.irf.com/product-info/datasheets/data/irf7343ipbf.pdf" H 5700 1550 50  0001 L CNN
	2    5600 1550
	1    0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 6452CB22
P 5400 2950
F 0 "R4" H 5470 2996 50  0000 L CNN
F 1 "100k" H 5470 2905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5330 2950 50  0001 C CNN
F 3 "~" H 5400 2950 50  0001 C CNN
	1    5400 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 6453988B
P 5400 1350
F 0 "R3" H 5150 1400 50  0000 L CNN
F 1 "100k" H 5150 1300 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5330 1350 50  0001 C CNN
F 3 "~" H 5400 1350 50  0001 C CNN
	1    5400 1350
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRF7343PBF Q1
U 1 1 6453F38D
P 4500 2750
F 0 "Q1" H 4705 2704 50  0000 L CNN
F 1 "IRF7343PBF" H 4705 2795 50  0001 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 4700 2675 50  0001 L CNN
F 3 "http://www.irf.com/product-info/datasheets/data/irf7343ipbf.pdf" H 4600 2750 50  0001 L CNN
	1    4500 2750
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 64542A76
P 4700 1350
F 0 "R1" H 4770 1396 50  0000 L CNN
F 1 "100k" H 4770 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4630 1350 50  0001 C CNN
F 3 "~" H 4700 1350 50  0001 C CNN
	1    4700 1350
	1    0    0    -1  
$EndComp
$Comp
L Diode:BAT54S D1
U 1 1 645433DA
P 4400 2050
F 0 "D1" V 4250 1850 50  0000 L CNN
F 1 "BAT54S" V 4300 1650 50  0001 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4475 2175 50  0001 L CNN
F 3 "https://www.diodes.com/assets/Datasheets/ds11005.pdf" H 4280 2050 50  0001 C CNN
	1    4400 2050
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:IRF7343PBF Q1
U 2 1 645443DB
P 4500 1550
F 0 "Q1" H 4705 1596 50  0000 L CNN
F 1 "IRF7343PBF" H 4705 1505 50  0001 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 4700 1475 50  0001 L CNN
F 3 "http://www.irf.com/product-info/datasheets/data/irf7343ipbf.pdf" H 4600 1550 50  0001 L CNN
	2    4500 1550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 64553303
P 4700 2950
F 0 "R2" H 4500 2900 50  0000 L CNN
F 1 "100k" H 4450 2800 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4630 2950 50  0001 C CNN
F 3 "~" H 4700 2950 50  0001 C CNN
	1    4700 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 1350 4400 1150
Wire Wire Line
	4400 1150 4700 1150
Wire Wire Line
	5700 1150 5700 1350
Wire Wire Line
	5400 1200 5400 1150
Connection ~ 5400 1150
Wire Wire Line
	5400 1150 5700 1150
Wire Wire Line
	4700 1200 4700 1150
Connection ~ 4700 1150
Wire Wire Line
	4700 1150 5050 1150
Wire Wire Line
	4700 1500 4700 1550
Wire Wire Line
	4700 1550 4750 1550
Connection ~ 4700 1550
Wire Wire Line
	5400 1500 5400 1550
Wire Wire Line
	5400 1550 5350 1550
Connection ~ 5400 1550
Wire Wire Line
	5050 1350 5050 1150
Connection ~ 5050 1150
Wire Wire Line
	5050 1150 5400 1150
Wire Wire Line
	4400 2950 4400 3150
Wire Wire Line
	4400 3150 4700 3150
Wire Wire Line
	5700 3150 5700 2950
Wire Wire Line
	5350 2750 5400 2750
Wire Wire Line
	5400 2750 5400 2800
Connection ~ 5400 2750
Wire Wire Line
	4700 2800 4700 2750
Wire Wire Line
	4700 2750 4750 2750
Connection ~ 4700 2750
Wire Wire Line
	4700 3100 4700 3150
Connection ~ 4700 3150
Wire Wire Line
	4700 3150 5050 3150
Wire Wire Line
	5400 3100 5400 3150
Connection ~ 5400 3150
Wire Wire Line
	5400 3150 5700 3150
Wire Wire Line
	5050 2950 5050 3150
Connection ~ 5050 3150
Wire Wire Line
	5050 3150 5400 3150
$Comp
L Device:C C1
U 1 1 6455CFC6
P 4700 1950
F 0 "C1" H 4815 1996 50  0000 L CNN
F 1 "82nF" H 4815 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4738 1800 50  0001 C CNN
F 3 "~" H 4700 1950 50  0001 C CNN
	1    4700 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 6455D756
P 4700 2600
F 0 "C2" H 4850 2700 50  0000 L CNN
F 1 "82nF" H 4850 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4738 2450 50  0001 C CNN
F 3 "~" H 4700 2600 50  0001 C CNN
	1    4700 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 6455E8D3
P 5400 2600
F 0 "C4" H 5150 2700 50  0000 L CNN
F 1 "82nF" H 5100 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5438 2450 50  0001 C CNN
F 3 "~" H 5400 2600 50  0001 C CNN
	1    5400 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 6455F066
P 5400 1950
F 0 "C3" H 5150 2000 50  0000 L CNN
F 1 "82nF" H 5100 1900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5438 1800 50  0001 C CNN
F 3 "~" H 5400 1950 50  0001 C CNN
	1    5400 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 645660BD
P 5050 2400
F 0 "R5" V 4850 2350 50  0000 L CNN
F 1 "10" V 4950 2350 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4980 2400 50  0001 C CNN
F 3 "~" H 5050 2400 50  0001 C CNN
	1    5050 2400
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 64566F42
P 3500 2750
F 0 "R6" V 3600 2700 50  0000 L CNN
F 1 "10" V 3700 2700 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 2750 50  0001 C CNN
F 3 "~" H 3500 2750 50  0001 C CNN
	1    3500 2750
	0    1    1    0   
$EndComp
$Comp
L Transistor_FET:IRF7343PBF Q2
U 1 1 6452BE0F
P 5600 2750
F 0 "Q2" H 5804 2704 50  0000 L CNN
F 1 "IRF7343PBF" H 5804 2795 50  0001 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 5800 2675 50  0001 L CNN
F 3 "http://www.irf.com/product-info/datasheets/data/irf7343ipbf.pdf" H 5700 2750 50  0001 L CNN
	1    5600 2750
	1    0    0    -1  
$EndComp
$Comp
L invertor-rescue:Conn-My_Library J4
U 1 1 645979CD
P 5800 1150
F 0 "J4" H 5828 1150 50  0001 L CNN
F 1 "Conn" H 5828 1105 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 5850 1000 50  0001 C CNN
F 3 "~" H 5800 1150 50  0001 C CNN
	1    5800 1150
	1    0    0    -1  
$EndComp
Connection ~ 5700 1150
$Comp
L invertor-rescue:Conn-My_Library J5
U 1 1 6459E72C
P 6000 2050
F 0 "J5" H 6028 2050 50  0001 L CNN
F 1 "Conn" H 6028 2005 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 6050 1900 50  0001 C CNN
F 3 "~" H 6000 2050 50  0001 C CNN
	1    6000 2050
	1    0    0    -1  
$EndComp
$Comp
L invertor-rescue:Conn-My_Library J1
U 1 1 6459EF77
P 4100 2050
F 0 "J1" H 4042 1957 50  0001 C CNN
F 1 "Conn" H 4128 2005 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 4150 1900 50  0001 C CNN
F 3 "~" H 4100 2050 50  0001 C CNN
	1    4100 2050
	-1   0    0    1   
$EndComp
$Comp
L invertor-rescue:Conn-My_Library J3
U 1 1 645A03DA
P 3250 2750
F 0 "J3" H 3278 2750 50  0001 L CNN
F 1 "Conn" H 3278 2705 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 3300 2600 50  0001 C CNN
F 3 "~" H 3250 2750 50  0001 C CNN
	1    3250 2750
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 645A16BE
P 5050 3150
F 0 "#PWR0101" H 5050 2900 50  0001 C CNN
F 1 "GND" H 5055 2977 50  0000 C CNN
F 2 "" H 5050 3150 50  0001 C CNN
F 3 "" H 5050 3150 50  0001 C CNN
	1    5050 3150
	1    0    0    -1  
$EndComp
$Comp
L invertor-rescue:Conn-My_Library J6
U 1 1 645A527F
P 5800 3150
F 0 "J6" H 5828 3150 50  0001 L CNN
F 1 "Conn" H 5828 3105 50  0001 L CNN
F 2 "My-library:SMD-CONN" H 5850 3000 50  0001 C CNN
F 3 "~" H 5800 3150 50  0001 C CNN
	1    5800 3150
	1    0    0    -1  
$EndComp
Connection ~ 5700 3150
Wire Wire Line
	4400 2350 4400 2550
Wire Wire Line
	5700 2350 5700 2550
Wire Wire Line
	5200 2400 5400 2400
Wire Wire Line
	5400 2400 5400 2450
Wire Wire Line
	4200 2050 4200 2400
Wire Wire Line
	4200 2400 4900 2400
Connection ~ 4200 2050
$Comp
L Device:Q_NMOS_GSD Q3
U 1 1 64672C0F
P 3900 2750
F 0 "Q3" H 3800 3100 50  0000 L CNN
F 1 "IRLML6344" H 3500 2950 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4100 2850 50  0001 C CNN
F 3 "~" H 3900 2750 50  0001 C CNN
	1    3900 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2950 4000 3150
$Comp
L Device:R R7
U 1 1 6467B7D9
P 4000 1350
F 0 "R7" H 4070 1396 50  0000 L CNN
F 1 "100" H 4070 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3930 1350 50  0001 C CNN
F 3 "~" H 4000 1350 50  0001 C CNN
	1    4000 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1150 4000 1200
Wire Wire Line
	4700 2450 4700 2100
Wire Wire Line
	4700 1800 4700 1550
Wire Wire Line
	5400 1800 5400 1550
Wire Wire Line
	5400 2400 5400 2100
Connection ~ 5400 2400
Wire Wire Line
	4000 1500 4000 2450
Wire Wire Line
	4000 3150 4400 3150
Connection ~ 4400 3150
Wire Wire Line
	4000 1150 4400 1150
Connection ~ 4400 1150
Wire Wire Line
	4700 2450 4000 2450
Connection ~ 4700 2450
Connection ~ 4000 2450
Wire Wire Line
	4000 2450 4000 2550
$Comp
L Device:R R8
U 1 1 646BA42D
P 3700 2950
F 0 "R8" H 3500 2900 50  0000 L CNN
F 1 "100k" H 3450 2800 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3630 2950 50  0001 C CNN
F 3 "~" H 3700 2950 50  0001 C CNN
	1    3700 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3100 3700 3150
Wire Wire Line
	3700 3150 4000 3150
Connection ~ 4000 3150
Wire Wire Line
	3650 2750 3700 2750
Wire Wire Line
	3700 2750 3700 2800
Connection ~ 3700 2750
$EndSCHEMATC
