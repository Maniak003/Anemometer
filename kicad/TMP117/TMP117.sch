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
L My_Library:TMP117 D1
U 1 1 623AF833
P 5450 2500
F 0 "D1" H 5600 2850 50  0000 C CNN
F 1 "TMP117" H 5500 2500 50  0000 C CNN
F 2 "My-library:TMP117" H 5250 2300 50  0001 C CNN
F 3 "" H 5250 2300 50  0001 C CNN
	1    5450 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 623AFD11
P 5450 3050
F 0 "#PWR0101" H 5450 2800 50  0001 C CNN
F 1 "GND" H 5455 2877 50  0000 C CNN
F 2 "" H 5450 3050 50  0001 C CNN
F 3 "" H 5450 3050 50  0001 C CNN
	1    5450 3050
	1    0    0    -1  
$EndComp
NoConn ~ 5850 2400
$Comp
L power:+3.3V #PWR0102
U 1 1 623B0130
P 5450 1850
F 0 "#PWR0102" H 5450 1700 50  0001 C CNN
F 1 "+3.3V" H 5600 1900 50  0000 C CNN
F 2 "" H 5450 1850 50  0001 C CNN
F 3 "" H 5450 1850 50  0001 C CNN
	1    5450 1850
	1    0    0    -1  
$EndComp
$Comp
L My_Library:Conn J1
U 1 1 623B0AA1
P 4800 1850
F 0 "J1" H 4742 1665 50  0001 C CNN
F 1 "Conn" H 4742 1756 50  0001 C CNN
F 2 "My-library:SMD-CONN" H 4850 1700 50  0001 C CNN
F 3 "~" H 4800 1850 50  0001 C CNN
	1    4800 1850
	-1   0    0    1   
$EndComp
$Comp
L My_Library:Conn J2
U 1 1 623B0F07
P 4800 2400
F 0 "J2" H 4742 2215 50  0001 C CNN
F 1 "Conn" H 4742 2306 50  0001 C CNN
F 2 "My-library:SMD-CONN" H 4850 2250 50  0001 C CNN
F 3 "~" H 4800 2400 50  0001 C CNN
	1    4800 2400
	-1   0    0    1   
$EndComp
$Comp
L My_Library:Conn J3
U 1 1 623B1137
P 4800 2650
F 0 "J3" H 4742 2465 50  0001 C CNN
F 1 "Conn" H 4742 2556 50  0001 C CNN
F 2 "My-library:SMD-CONN" H 4850 2500 50  0001 C CNN
F 3 "~" H 4800 2650 50  0001 C CNN
	1    4800 2650
	-1   0    0    1   
$EndComp
$Comp
L My_Library:Conn J4
U 1 1 623B1419
P 4800 3050
F 0 "J4" H 4742 2865 50  0001 C CNN
F 1 "Conn" H 4742 2956 50  0001 C CNN
F 2 "My-library:SMD-CONN" H 4850 2900 50  0001 C CNN
F 3 "~" H 4800 3050 50  0001 C CNN
	1    4800 3050
	-1   0    0    1   
$EndComp
Wire Wire Line
	4900 3050 5450 3050
Wire Wire Line
	4900 2650 4950 2650
Wire Wire Line
	4900 2400 5100 2400
Wire Wire Line
	4900 1850 4950 1850
$Comp
L Device:R R1
U 1 1 623B3B8B
P 4950 2100
F 0 "R1" H 4750 2150 50  0000 L CNN
F 1 "5k" H 4750 2050 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4880 2100 50  0001 C CNN
F 3 "~" H 4950 2100 50  0001 C CNN
	1    4950 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 623B6D0A
P 5100 2100
F 0 "R2" H 5170 2146 50  0000 L CNN
F 1 "5k" H 5170 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 5030 2100 50  0001 C CNN
F 3 "~" H 5100 2100 50  0001 C CNN
	1    5100 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2900 5450 3050
Connection ~ 5450 3050
Wire Wire Line
	5450 3050 5850 3050
Wire Wire Line
	5850 2600 5850 3050
Wire Wire Line
	5100 2250 5100 2400
Connection ~ 5100 2400
Wire Wire Line
	4950 2250 4950 2650
Connection ~ 4950 2650
Wire Wire Line
	4950 2650 5100 2650
Wire Wire Line
	4950 1950 4950 1850
Connection ~ 4950 1850
Wire Wire Line
	4950 1850 5100 1850
Wire Wire Line
	5100 1950 5100 1850
Connection ~ 5100 1850
Wire Wire Line
	5100 1850 5450 1850
Wire Wire Line
	5450 1850 5450 2150
Connection ~ 5450 1850
$EndSCHEMATC
