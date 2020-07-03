EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Green Pill Bis"
Date ""
Rev "a"
Comp "Lars Boegild Thomsen"
Comment1 "STM32 Development Board"
Comment2 "lbthomsen@gmail.com"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_ST_STM32F1:STM32F103C8Tx U2
U 1 1 5EF49305
P 7850 2650
F 0 "U2" H 7300 4100 50  0000 C CNN
F 1 "STM32F103C8T6" H 8400 4100 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 7250 1250 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 7850 2650 50  0001 C CNN
F 4 "C8734" H 7850 2650 50  0001 C CNN "LCSC"
	1    7850 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5EF4C463
P 2800 1350
F 0 "C6" H 2800 1450 50  0000 L CNN
F 1 "100nF" H 2800 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2838 1200 50  0001 C CNN
F 3 "~" H 2800 1350 50  0001 C CNN
F 4 "C1525" H 2800 1350 50  0001 C CNN "LCSC"
	1    2800 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5EF4D338
P 750 1400
F 0 "C1" H 750 1500 50  0000 L CNN
F 1 "10uF" H 750 1300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 788 1250 50  0001 C CNN
F 3 "~" H 750 1400 50  0001 C CNN
F 4 "C15850" H 750 1400 50  0001 C CNN "LCSC"
	1    750  1400
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 5EF50EEB
P 3050 2950
F 0 "D1" H 3050 2825 50  0000 C CNN
F 1 "~" H 3050 2824 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 3050 2950 50  0001 C CNN
F 3 "~" H 3050 2950 50  0001 C CNN
F 4 "C8598" H 3050 2950 50  0001 C CNN "LCSC"
	1    3050 2950
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5EF57C35
P 1550 1400
F 0 "C2" H 1550 1500 50  0000 L CNN
F 1 "10uF" H 1550 1300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1588 1250 50  0001 C CNN
F 3 "~" H 1550 1400 50  0001 C CNN
F 4 "C15850" H 1550 1400 50  0001 C CNN "LCSC"
	1    1550 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5EF5A711
P 3050 1350
F 0 "C8" H 3050 1450 50  0000 L CNN
F 1 "100nF" H 3050 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3088 1200 50  0001 C CNN
F 3 "~" H 3050 1350 50  0001 C CNN
F 4 "C1525" H 3050 1350 50  0001 C CNN "LCSC"
	1    3050 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5EF5AA94
P 3300 1350
F 0 "C9" H 3300 1450 50  0000 L CNN
F 1 "100nF" H 3300 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3338 1200 50  0001 C CNN
F 3 "~" H 3300 1350 50  0001 C CNN
F 4 "C1525" H 3300 1350 50  0001 C CNN "LCSC"
	1    3300 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5EF5AEC1
P 3550 1350
F 0 "C10" H 3550 1450 50  0000 L CNN
F 1 "100nF" H 3550 1250 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3588 1200 50  0001 C CNN
F 3 "~" H 3550 1350 50  0001 C CNN
F 4 "C1525" H 3550 1350 50  0001 C CNN "LCSC"
	1    3550 1350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x20 J2
U 1 1 5EF5C946
P 10750 1800
F 0 "J2" H 10830 1746 50  0000 L CNN
F 1 "~" H 10830 1701 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 10750 1800 50  0001 C CNN
F 3 "~" H 10750 1800 50  0001 C CNN
	1    10750 1800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x20 J3
U 1 1 5EF5DEF5
P 10750 4450
F 0 "J3" H 10830 4396 50  0000 L CNN
F 1 "~" H 10830 4351 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 10750 4450 50  0001 C CNN
F 3 "~" H 10750 4450 50  0001 C CNN
	1    10750 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5EF6A604
P 1000 4650
F 0 "#PWR03" H 1000 4400 50  0001 C CNN
F 1 "GND" H 1005 4477 50  0000 C CNN
F 2 "" H 1000 4650 50  0001 C CNN
F 3 "" H 1000 4650 50  0001 C CNN
	1    1000 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  1250 750  1150
Connection ~ 750  1150
Wire Wire Line
	750  1150 850  1150
$Comp
L power:GND #PWR02
U 1 1 5EF6D754
P 750 1650
F 0 "#PWR02" H 750 1400 50  0001 C CNN
F 1 "GND" H 755 1477 50  0000 C CNN
F 2 "" H 750 1650 50  0001 C CNN
F 3 "" H 750 1650 50  0001 C CNN
	1    750  1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  1650 750  1550
$Comp
L power:GND #PWR06
U 1 1 5EF6DD55
P 1150 1650
F 0 "#PWR06" H 1150 1400 50  0001 C CNN
F 1 "GND" H 1155 1477 50  0000 C CNN
F 2 "" H 1150 1650 50  0001 C CNN
F 3 "" H 1150 1650 50  0001 C CNN
	1    1150 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1650 1150 1550
$Comp
L power:GND #PWR08
U 1 1 5EF6E42A
P 1550 1650
F 0 "#PWR08" H 1550 1400 50  0001 C CNN
F 1 "GND" H 1555 1477 50  0000 C CNN
F 2 "" H 1550 1650 50  0001 C CNN
F 3 "" H 1550 1650 50  0001 C CNN
	1    1550 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5EF6EC94
P 2800 1600
F 0 "#PWR018" H 2800 1350 50  0001 C CNN
F 1 "GND" H 2805 1427 50  0000 C CNN
F 2 "" H 2800 1600 50  0001 C CNN
F 3 "" H 2800 1600 50  0001 C CNN
	1    2800 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5EF6F043
P 3050 1600
F 0 "#PWR020" H 3050 1350 50  0001 C CNN
F 1 "GND" H 3055 1427 50  0000 C CNN
F 2 "" H 3050 1600 50  0001 C CNN
F 3 "" H 3050 1600 50  0001 C CNN
	1    3050 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5EF6F301
P 3300 1600
F 0 "#PWR021" H 3300 1350 50  0001 C CNN
F 1 "GND" H 3305 1427 50  0000 C CNN
F 2 "" H 3300 1600 50  0001 C CNN
F 3 "" H 3300 1600 50  0001 C CNN
	1    3300 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5EF6F66E
P 3550 1600
F 0 "#PWR022" H 3550 1350 50  0001 C CNN
F 1 "GND" H 3555 1427 50  0000 C CNN
F 2 "" H 3550 1600 50  0001 C CNN
F 3 "" H 3550 1600 50  0001 C CNN
	1    3550 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1650 1550 1550
Wire Wire Line
	2800 1600 2800 1500
Wire Wire Line
	3050 1600 3050 1500
Wire Wire Line
	3300 1600 3300 1500
Wire Wire Line
	3550 1600 3550 1500
Wire Wire Line
	1450 1150 1550 1150
Wire Wire Line
	3550 1100 3550 1200
Wire Wire Line
	3300 1200 3300 1100
Connection ~ 3300 1100
Wire Wire Line
	3300 1100 3550 1100
Wire Wire Line
	3050 1200 3050 1100
Connection ~ 3050 1100
Wire Wire Line
	3050 1100 3300 1100
Wire Wire Line
	2800 1200 2800 1100
Wire Wire Line
	2800 1100 3050 1100
Wire Wire Line
	1550 1250 1550 1150
Connection ~ 1550 1150
$Comp
L power:+3.3V #PWR017
U 1 1 5EF7A957
P 2800 1000
F 0 "#PWR017" H 2800 850 50  0001 C CNN
F 1 "+3.3V" H 2815 1173 50  0000 C CNN
F 2 "" H 2800 1000 50  0001 C CNN
F 3 "" H 2800 1000 50  0001 C CNN
	1    2800 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  1150 750  1050
$Comp
L power:+3.3V #PWR029
U 1 1 5EF7BF8B
P 7850 950
F 0 "#PWR029" H 7850 800 50  0001 C CNN
F 1 "+3.3V" H 7865 1123 50  0000 C CNN
F 2 "" H 7850 950 50  0001 C CNN
F 3 "" H 7850 950 50  0001 C CNN
	1    7850 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 1150 7650 1050
Wire Wire Line
	7650 1050 7750 1050
Wire Wire Line
	8050 1050 8050 1150
Wire Wire Line
	7850 950  7850 1050
Connection ~ 7850 1050
Wire Wire Line
	7850 1050 7950 1050
Wire Wire Line
	7750 1150 7750 1050
Connection ~ 7750 1050
Wire Wire Line
	7750 1050 7850 1050
Wire Wire Line
	7850 1150 7850 1050
Wire Wire Line
	7950 1150 7950 1050
Connection ~ 7950 1050
Wire Wire Line
	7950 1050 8050 1050
Text GLabel 5550 2050 0    50   Input ~ 0
C13
Text GLabel 5550 2150 0    50   Input ~ 0
C14
Text GLabel 5550 2250 0    50   Input ~ 0
C15
Text GLabel 7050 1550 0    50   Input ~ 0
BO0
$Comp
L power:GND #PWR028
U 1 1 5EF8A264
P 7800 4350
F 0 "#PWR028" H 7800 4100 50  0001 C CNN
F 1 "GND" H 7805 4177 50  0000 C CNN
F 2 "" H 7800 4350 50  0001 C CNN
F 3 "" H 7800 4350 50  0001 C CNN
	1    7800 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 3450 1700 3450
Wire Wire Line
	7050 1350 7150 1350
Wire Wire Line
	7050 1550 7150 1550
Wire Wire Line
	5550 2050 7150 2050
Wire Wire Line
	5550 2150 5800 2150
Wire Wire Line
	5550 2250 6300 2250
Text GLabel 7050 2450 0    50   Input ~ 0
B0
Text GLabel 7050 2550 0    50   Input ~ 0
B1
Text GLabel 7050 2750 0    50   Input ~ 0
B3
Text GLabel 7050 2850 0    50   Input ~ 0
B4
Text GLabel 7050 2950 0    50   Input ~ 0
B5
Text GLabel 7050 3050 0    50   Input ~ 0
B6
Text GLabel 7050 3150 0    50   Input ~ 0
B7
Text GLabel 7050 3250 0    50   Input ~ 0
B8
Text GLabel 7050 3350 0    50   Input ~ 0
B9
Text GLabel 7050 3450 0    50   Input ~ 0
B10
Text GLabel 7050 3550 0    50   Input ~ 0
B11
Text GLabel 7050 3650 0    50   Input ~ 0
B12
Text GLabel 7050 3750 0    50   Input ~ 0
B13
Text GLabel 7050 3850 0    50   Input ~ 0
B14
Text GLabel 7050 3950 0    50   Input ~ 0
B15
Text GLabel 8550 2450 2    50   Input ~ 0
A0
Text GLabel 8550 2550 2    50   Input ~ 0
A1
Text GLabel 8550 2650 2    50   Input ~ 0
A2
Text GLabel 8550 2750 2    50   Input ~ 0
A3
Text GLabel 8550 2850 2    50   Input ~ 0
A4
Text GLabel 8550 2950 2    50   Input ~ 0
A5
Text GLabel 8550 3050 2    50   Input ~ 0
A6
Text GLabel 8550 3150 2    50   Input ~ 0
A7
Text GLabel 8550 3250 2    50   Input ~ 0
A8
Text GLabel 8550 3350 2    50   Input ~ 0
A9
Text GLabel 8550 3450 2    50   Input ~ 0
A10
Text GLabel 8550 3550 2    50   Input ~ 0
A11
Text GLabel 8550 3650 2    50   Input ~ 0
A12
Text GLabel 8550 3750 2    50   Input ~ 0
SWDIO
Text GLabel 8550 3850 2    50   Input ~ 0
SWCLK
Text GLabel 8550 3950 2    50   Input ~ 0
A15
Wire Wire Line
	8450 2450 8550 2450
Wire Wire Line
	8550 2550 8450 2550
Wire Wire Line
	8450 2650 8550 2650
Wire Wire Line
	8550 2750 8450 2750
Wire Wire Line
	8450 2850 8550 2850
Wire Wire Line
	8550 2950 8450 2950
Wire Wire Line
	8450 3050 8550 3050
Wire Wire Line
	8550 3150 8450 3150
Wire Wire Line
	8450 3250 8550 3250
Wire Wire Line
	8450 3350 8550 3350
Wire Wire Line
	8550 3450 8450 3450
Wire Wire Line
	8450 3550 8550 3550
Wire Wire Line
	8550 3650 8450 3650
Wire Wire Line
	8450 3750 8550 3750
Wire Wire Line
	8550 3850 8450 3850
Wire Wire Line
	8450 3950 8550 3950
Wire Wire Line
	7150 2450 7050 2450
Wire Wire Line
	7050 2550 7150 2550
Wire Wire Line
	7050 2650 7150 2650
Wire Wire Line
	7150 2750 7050 2750
Wire Wire Line
	7050 2850 7150 2850
Wire Wire Line
	7150 2950 7050 2950
Wire Wire Line
	7050 3050 7150 3050
Wire Wire Line
	7150 3150 7050 3150
Wire Wire Line
	7050 3250 7150 3250
Wire Wire Line
	7150 3350 7050 3350
Wire Wire Line
	7050 3450 7150 3450
Wire Wire Line
	7150 3550 7050 3550
Wire Wire Line
	7050 3650 7150 3650
Wire Wire Line
	7150 3750 7050 3750
Wire Wire Line
	7050 3850 7150 3850
Wire Wire Line
	7150 3950 7050 3950
Wire Wire Line
	7650 4150 7650 4250
Wire Wire Line
	7650 4250 7750 4250
Wire Wire Line
	7950 4250 7950 4150
Wire Wire Line
	7800 4350 7800 4250
Connection ~ 7800 4250
Wire Wire Line
	7800 4250 7850 4250
Wire Wire Line
	7750 4150 7750 4250
Connection ~ 7750 4250
Wire Wire Line
	7750 4250 7800 4250
Wire Wire Line
	7850 4150 7850 4250
Connection ~ 7850 4250
Wire Wire Line
	7850 4250 7950 4250
Text GLabel 9900 1000 0    50   Input ~ 0
C13
Text GLabel 9900 1100 0    50   Input ~ 0
C14
Text GLabel 9900 1200 0    50   Input ~ 0
C15
Wire Wire Line
	9900 1000 10550 1000
Wire Wire Line
	10550 1100 9900 1100
Wire Wire Line
	9900 1200 10550 1200
Text GLabel 9900 1300 0    50   Input ~ 0
A0
Text GLabel 9900 1400 0    50   Input ~ 0
A1
Text GLabel 9900 1500 0    50   Input ~ 0
A2
Wire Wire Line
	9900 1300 10550 1300
Wire Wire Line
	10550 1400 9900 1400
Wire Wire Line
	9900 1500 10550 1500
Text GLabel 10050 3850 0    50   Input ~ 0
B9
Text GLabel 10050 3950 0    50   Input ~ 0
B8
Text GLabel 10050 4050 0    50   Input ~ 0
B7
Wire Wire Line
	10050 3850 10550 3850
Text GLabel 10050 4150 0    50   Input ~ 0
B6
Text GLabel 10050 4250 0    50   Input ~ 0
B5
Text GLabel 10050 4450 0    50   Input ~ 0
B3
Wire Wire Line
	10050 3950 10550 3950
Wire Wire Line
	10550 4050 10050 4050
Wire Wire Line
	10050 4150 10550 4150
Wire Wire Line
	10050 4250 10550 4250
Text GLabel 10050 4550 0    50   Input ~ 0
A15
Wire Wire Line
	10050 4350 10550 4350
Wire Wire Line
	10050 4450 10550 4450
Text GLabel 10050 4650 0    50   Input ~ 0
A12
Text GLabel 10050 4750 0    50   Input ~ 0
A11
Wire Wire Line
	10050 4550 10550 4550
Wire Wire Line
	10550 4650 10050 4650
Wire Wire Line
	10050 4750 10550 4750
Text GLabel 10050 4850 0    50   Input ~ 0
A10
Text GLabel 10050 4950 0    50   Input ~ 0
A9
Text GLabel 10050 5050 0    50   Input ~ 0
A8
Wire Wire Line
	10050 4850 10550 4850
Wire Wire Line
	10050 4950 10550 4950
Wire Wire Line
	10050 5050 10550 5050
Text GLabel 10050 5150 0    50   Input ~ 0
B15
Text GLabel 10050 5250 0    50   Input ~ 0
B14
Text GLabel 10050 5350 0    50   Input ~ 0
B13
Text GLabel 10050 5450 0    50   Input ~ 0
B12
Wire Wire Line
	10050 5150 10550 5150
Wire Wire Line
	10050 5250 10550 5250
Wire Wire Line
	10550 5350 10050 5350
Wire Wire Line
	10050 5450 10550 5450
Wire Wire Line
	9900 1600 10550 1600
Text GLabel 9900 1600 0    50   Input ~ 0
A3
Text GLabel 9900 1700 0    50   Input ~ 0
A4
Text GLabel 9900 1800 0    50   Input ~ 0
A5
Text GLabel 9900 1900 0    50   Input ~ 0
A6
Text GLabel 9900 2000 0    50   Input ~ 0
A7
Wire Wire Line
	9900 2000 10550 2000
Wire Wire Line
	10550 1900 9900 1900
Wire Wire Line
	9900 1800 10550 1800
Wire Wire Line
	10550 1700 9900 1700
Text GLabel 9900 2100 0    50   Input ~ 0
B0
Text GLabel 9900 2200 0    50   Input ~ 0
B1
Text GLabel 9900 2400 0    50   Input ~ 0
B10
Text GLabel 9900 2500 0    50   Input ~ 0
B11
Wire Wire Line
	9900 2100 10550 2100
Wire Wire Line
	10550 2200 9900 2200
Wire Wire Line
	9900 2300 10550 2300
Wire Wire Line
	9900 2400 10550 2400
Text GLabel 2650 3650 2    50   Input ~ 0
A12
Text GLabel 2650 3450 2    50   Input ~ 0
A11
Text GLabel 10050 4350 0    50   Input ~ 0
B4
$Comp
L Connector_Generic:Conn_01x04 J4
U 1 1 5EFA63FB
P 5900 6450
F 0 "J4" H 5980 6396 50  0000 L CNN
F 1 "~" H 5980 6351 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 5900 6450 50  0001 C CNN
F 3 "~" H 5900 6450 50  0001 C CNN
	1    5900 6450
	1    0    0    -1  
$EndComp
Text GLabel 5500 6550 0    50   Input ~ 0
SWCLK
Text GLabel 5500 6450 0    50   Input ~ 0
SWDIO
Wire Wire Line
	5500 6450 5700 6450
Wire Wire Line
	5700 6550 5500 6550
$Comp
L power:+3.3V #PWR036
U 1 1 5F0315DC
P 5600 6250
F 0 "#PWR036" H 5600 6100 50  0001 C CNN
F 1 "+3.3V" H 5615 6423 50  0000 C CNN
F 2 "" H 5600 6250 50  0001 C CNN
F 3 "" H 5600 6250 50  0001 C CNN
	1    5600 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR037
U 1 1 5F032456
P 5600 6750
F 0 "#PWR037" H 5600 6500 50  0001 C CNN
F 1 "GND" H 5605 6577 50  0000 C CNN
F 2 "" H 5600 6750 50  0001 C CNN
F 3 "" H 5600 6750 50  0001 C CNN
	1    5600 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 6650 5600 6650
Wire Wire Line
	5600 6650 5600 6750
Wire Wire Line
	5700 6350 5600 6350
Wire Wire Line
	5600 6350 5600 6250
$Comp
L Switch:SW_Push SW1
U 1 1 5F055544
P 1050 6400
F 0 "SW1" H 950 6500 50  0000 C CNN
F 1 "~" H 1250 6500 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3342" H 1050 6600 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/1901111503_XKB-Enterprise-TS-1187A-B-A-B_C318884.pdf" H 1050 6600 50  0001 C CNN
F 4 "C318884" H 1050 6400 50  0001 C CNN "LCSC"
	1    1050 6400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5F055552
P 1300 6750
F 0 "#PWR010" H 1300 6500 50  0001 C CNN
F 1 "GND" H 1305 6577 50  0000 C CNN
F 2 "" H 1300 6750 50  0001 C CNN
F 3 "" H 1300 6750 50  0001 C CNN
	1    1300 6750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F066105
P 1300 6400
F 0 "C3" H 1300 6500 50  0000 L CNN
F 1 "100nF" H 1300 6300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1338 6250 50  0001 C CNN
F 3 "~" H 1300 6400 50  0001 C CNN
F 4 "C1525" H 1300 6400 50  0001 C CNN "LCSC"
	1    1300 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 6200 1300 6200
Connection ~ 1300 6200
Wire Wire Line
	1300 6200 1300 6250
Wire Wire Line
	1050 6600 1050 6650
Wire Wire Line
	1050 6650 1300 6650
Wire Wire Line
	1300 6650 1300 6550
Wire Wire Line
	1300 6750 1300 6650
Connection ~ 1300 6650
Text GLabel 1500 6200 2    50   Input ~ 0
RST
Wire Wire Line
	1500 6200 1300 6200
Text GLabel 7050 1350 0    50   Input ~ 0
RST
$Comp
L Device:LED D2
U 1 1 5EF16E42
P 3650 6300
F 0 "D2" V 3750 6400 50  0000 R CNN
F 1 "YEL" V 3550 6450 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3650 6300 50  0001 C CNN
F 3 "~" H 3650 6300 50  0001 C CNN
F 4 "C72038" V 3650 6300 50  0001 C CNN "LCSC"
	1    3650 6300
	0    1    -1   0   
$EndComp
$Comp
L Device:R R6
U 1 1 5EF4D4A5
P 3650 6700
F 0 "R6" H 3750 6850 50  0000 C CNN
F 1 "1k" H 3750 6550 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3580 6700 50  0001 C CNN
F 3 "~" H 3650 6700 50  0001 C CNN
F 4 "C25905" V 3650 6700 50  0001 C CNN "LCSC"
	1    3650 6700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5EF54866
P 3650 6950
F 0 "#PWR025" H 3650 6700 50  0001 C CNN
F 1 "GND" H 3655 6777 50  0000 C CNN
F 2 "" H 3650 6950 50  0001 C CNN
F 3 "" H 3650 6950 50  0001 C CNN
	1    3650 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 6150 3650 6050
Wire Wire Line
	3650 6550 3650 6450
Wire Wire Line
	3650 6950 3650 6850
$Comp
L Device:R R7
U 1 1 5EF69058
P 3900 6700
F 0 "R7" H 4000 6850 50  0000 C CNN
F 1 "1k" H 4000 6550 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3830 6700 50  0001 C CNN
F 3 "~" H 3900 6700 50  0001 C CNN
F 4 "C25905" V 3900 6700 50  0001 C CNN "LCSC"
	1    3900 6700
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5EF6934F
P 3900 6300
F 0 "D3" V 4000 6400 50  0000 R CNN
F 1 "BLUE" V 3800 6450 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 3900 6300 50  0001 C CNN
F 3 "~" H 3900 6300 50  0001 C CNN
F 4 "C72041" V 3900 6300 50  0001 C CNN "LCSC"
	1    3900 6300
	0    1    -1   0   
$EndComp
Text GLabel 4000 7000 2    50   Input ~ 0
C13
$Comp
L power:+3.3V #PWR026
U 1 1 5EF6A3B0
P 3900 6050
F 0 "#PWR026" H 3900 5900 50  0001 C CNN
F 1 "+3.3V" H 3915 6223 50  0000 C CNN
F 2 "" H 3900 6050 50  0001 C CNN
F 3 "" H 3900 6050 50  0001 C CNN
	1    3900 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6050 3900 6150
Wire Wire Line
	3900 6450 3900 6550
Wire Wire Line
	3900 6850 3900 7000
Wire Wire Line
	3900 7000 4000 7000
$Comp
L Device:R R4
U 1 1 5EFA4627
P 3100 3500
F 0 "R4" H 3100 3650 50  0000 L CNN
F 1 "5.1k" H 3100 3350 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3030 3500 50  0001 C CNN
F 3 "~" H 3100 3500 50  0001 C CNN
F 4 "C25905" H 3100 3500 50  0001 C CNN "LCSC"
	1    3100 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR024
U 1 1 5EFBAD18
P 3650 6050
F 0 "#PWR024" H 3650 5900 50  0001 C CNN
F 1 "+3.3V" H 3665 6223 50  0000 C CNN
F 2 "" H 3650 6050 50  0001 C CNN
F 3 "" H 3650 6050 50  0001 C CNN
	1    3650 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5EFC3FFE
P 4450 2200
F 0 "C5" H 4450 2300 50  0000 L CNN
F 1 "20pF" H 4450 2100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4488 2050 50  0001 C CNN
F 3 "~" H 4450 2200 50  0001 C CNN
F 4 "C1554" H 4450 2200 50  0001 C CNN "LCSC"
	1    4450 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y2
U 1 1 5EFC51BA
P 6050 2450
F 0 "Y2" H 5900 2550 50  0000 C CNN
F 1 "32.768k" H 6250 2550 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_EuroQuartz_EQ161-2Pin_3.2x1.5mm" H 6050 2450 50  0001 C CNN
F 3 "~" H 6050 2450 50  0001 C CNN
F 4 "C32346" H 6050 2450 50  0001 C CNN "LCSC"
	1    6050 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 5EFDAC67
P 4700 1950
F 0 "Y1" H 4550 2050 50  0000 C CNN
F 1 "8M" H 4850 2050 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_Abracon_ABM3-2Pin_5.0x3.2mm" H 4700 1950 50  0001 C CNN
F 3 "~" H 4700 1950 50  0001 C CNN
F 4 "C115962" H 4700 1950 50  0001 C CNN "LCSC"
	1    4700 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5EFDB436
P 4950 2200
F 0 "C7" H 4950 2300 50  0000 L CNN
F 1 "20pF" H 4950 2100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4988 2050 50  0001 C CNN
F 3 "~" H 4950 2200 50  0001 C CNN
F 4 "C1554" H 4950 2200 50  0001 C CNN "LCSC"
	1    4950 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5EFDBAA6
P 4450 2450
F 0 "#PWR016" H 4450 2200 50  0001 C CNN
F 1 "GND" H 4455 2277 50  0000 C CNN
F 2 "" H 4450 2450 50  0001 C CNN
F 3 "" H 4450 2450 50  0001 C CNN
	1    4450 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5EFDC2D9
P 4950 2450
F 0 "#PWR019" H 4950 2200 50  0001 C CNN
F 1 "GND" H 4955 2277 50  0000 C CNN
F 2 "" H 4950 2450 50  0001 C CNN
F 3 "" H 4950 2450 50  0001 C CNN
	1    4950 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1950 4450 1950
Wire Wire Line
	4450 1950 4450 2050
Wire Wire Line
	4850 1950 4950 1950
Wire Wire Line
	4950 1950 4950 2050
Wire Wire Line
	4950 2350 4950 2450
Wire Wire Line
	4450 2350 4450 2450
Wire Wire Line
	7150 1750 4450 1750
Wire Wire Line
	4450 1750 4450 1950
Connection ~ 4450 1950
Wire Wire Line
	4950 1950 4950 1850
Wire Wire Line
	4950 1850 7150 1850
Connection ~ 4950 1950
$Comp
L Device:C C11
U 1 1 5F056926
P 5800 2700
F 0 "C11" H 5800 2800 50  0000 L CNN
F 1 "20pF" H 5800 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5838 2550 50  0001 C CNN
F 3 "~" H 5800 2700 50  0001 C CNN
F 4 "C1554" H 5800 2700 50  0001 C CNN "LCSC"
	1    5800 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5F056F51
P 6300 2700
F 0 "C12" H 6300 2800 50  0000 L CNN
F 1 "20pF" H 6300 2600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6338 2550 50  0001 C CNN
F 3 "~" H 6300 2700 50  0001 C CNN
F 4 "C1554" H 6300 2700 50  0001 C CNN "LCSC"
	1    6300 2700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 5F057B0D
P 5800 2950
F 0 "#PWR023" H 5800 2700 50  0001 C CNN
F 1 "GND" H 5805 2777 50  0000 C CNN
F 2 "" H 5800 2950 50  0001 C CNN
F 3 "" H 5800 2950 50  0001 C CNN
	1    5800 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 5F057E46
P 6300 2950
F 0 "#PWR027" H 6300 2700 50  0001 C CNN
F 1 "GND" H 6305 2777 50  0000 C CNN
F 2 "" H 6300 2950 50  0001 C CNN
F 3 "" H 6300 2950 50  0001 C CNN
	1    6300 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 2850 5800 2950
Wire Wire Line
	6300 2950 6300 2850
Wire Wire Line
	5800 2550 5800 2450
Connection ~ 5800 2150
Wire Wire Line
	5800 2150 7150 2150
Wire Wire Line
	5900 2450 5800 2450
Connection ~ 5800 2450
Wire Wire Line
	5800 2450 5800 2150
Connection ~ 6300 2250
Wire Wire Line
	6300 2250 7150 2250
Wire Wire Line
	6200 2450 6300 2450
Connection ~ 6300 2450
Wire Wire Line
	6300 2450 6300 2250
Wire Wire Line
	6300 2450 6300 2550
$Comp
L Connector:USB_C_Receptacle_USB2.0 J1
U 1 1 5EF5BC4B
P 1000 3550
F 0 "J1" H 1107 4417 50  0000 C CNN
F 1 "USB_C_Receptacle_USB2.0" H 1107 4326 50  0000 C CNN
F 2 "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12" H 1150 3550 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/1811101520_Jing-Extension-of-the-Electronic-Co-C167321_C167321.pdf" H 1150 3550 50  0001 C CNN
F 4 "C167321" H 1000 3550 50  0001 C CNN "LCSC"
	1    1000 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 3550 1700 3550
Wire Wire Line
	1700 3550 1700 3450
Connection ~ 1700 3450
Wire Wire Line
	1600 3750 1700 3750
Wire Wire Line
	1700 3750 1700 3650
Connection ~ 1700 3650
Wire Wire Line
	1700 3650 1600 3650
Wire Wire Line
	10450 900  10550 900 
$Comp
L power:+3.3V #PWR031
U 1 1 5EF5F475
P 10000 800
F 0 "#PWR031" H 10000 650 50  0001 C CNN
F 1 "+3.3V" H 10015 973 50  0000 C CNN
F 2 "" H 10000 800 50  0001 C CNN
F 3 "" H 10000 800 50  0001 C CNN
	1    10000 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 2600 10550 2600
Wire Wire Line
	10000 800  10000 2600
Wire Wire Line
	9900 2500 10550 2500
$Comp
L power:GND #PWR034
U 1 1 5EF9371A
P 10450 2900
F 0 "#PWR034" H 10450 2650 50  0001 C CNN
F 1 "GND" H 10455 2727 50  0000 C CNN
F 2 "" H 10450 2900 50  0001 C CNN
F 3 "" H 10450 2900 50  0001 C CNN
	1    10450 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 2900 10450 2800
Wire Wire Line
	10550 2800 10450 2800
$Comp
L Device:R R5
U 1 1 5EF5EBD1
P 3300 3500
F 0 "R5" H 3300 3650 50  0000 L CNN
F 1 "5.1k" H 3300 3350 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 3230 3500 50  0001 C CNN
F 3 "~" H 3300 3500 50  0001 C CNN
F 4 "C25905" H 3300 3500 50  0001 C CNN "LCSC"
	1    3300 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5EF5F11B
P 3100 3750
F 0 "#PWR011" H 3100 3500 50  0001 C CNN
F 1 "GND" H 3105 3577 50  0000 C CNN
F 2 "" H 3100 3750 50  0001 C CNN
F 3 "" H 3100 3750 50  0001 C CNN
	1    3100 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5EF5F5EE
P 3300 3750
F 0 "#PWR013" H 3300 3500 50  0001 C CNN
F 1 "GND" H 3305 3577 50  0000 C CNN
F 2 "" H 3300 3750 50  0001 C CNN
F 3 "" H 3300 3750 50  0001 C CNN
	1    3300 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2950 1850 2950
Wire Wire Line
	3100 3650 3100 3750
Wire Wire Line
	3300 3750 3300 3650
Wire Wire Line
	3100 3350 3100 3250
Wire Wire Line
	3100 3250 1600 3250
Wire Wire Line
	1600 3150 3300 3150
Wire Wire Line
	3300 3150 3300 3350
$Comp
L power:GND #PWR035
U 1 1 5EF95054
P 10450 5550
F 0 "#PWR035" H 10450 5300 50  0001 C CNN
F 1 "GND" H 10455 5377 50  0000 C CNN
F 2 "" H 10450 5550 50  0001 C CNN
F 3 "" H 10450 5550 50  0001 C CNN
	1    10450 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 4450 1000 4550
NoConn ~ 13000 6350
Wire Wire Line
	700  4450 700  4550
Wire Wire Line
	700  4550 1000 4550
Connection ~ 1000 4550
Wire Wire Line
	1000 4550 1000 4650
$Comp
L Switch:SW_Push SW2
U 1 1 5EFDDDE0
P 1900 6400
F 0 "SW2" H 1800 6500 50  0000 C CNN
F 1 "~" H 2100 6500 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3342" H 1900 6600 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/1901111503_XKB-Enterprise-TS-1187A-B-A-B_C318884.pdf" H 1900 6600 50  0001 C CNN
F 4 "C318884" H 1900 6400 50  0001 C CNN "LCSC"
	1    1900 6400
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C4
U 1 1 5EFDEFAE
P 2100 6400
F 0 "C4" H 2100 6500 50  0000 L CNN
F 1 "100nF" H 2100 6300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2138 6250 50  0001 C CNN
F 3 "~" H 2100 6400 50  0001 C CNN
F 4 "C1525" H 2100 6400 50  0001 C CNN "LCSC"
	1    2100 6400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 5EFDFFB8
P 2100 6150
F 0 "#PWR014" H 2100 6000 50  0001 C CNN
F 1 "+3.3V" H 2115 6323 50  0000 C CNN
F 2 "" H 2100 6150 50  0001 C CNN
F 3 "" H 2100 6150 50  0001 C CNN
	1    2100 6150
	1    0    0    -1  
$EndComp
Text GLabel 2200 6650 2    50   Input ~ 0
BO0
Wire Wire Line
	2100 6150 2100 6200
Wire Wire Line
	1900 6200 2100 6200
Connection ~ 2100 6200
Wire Wire Line
	2100 6200 2100 6250
Wire Wire Line
	1900 6600 1900 6650
Wire Wire Line
	1900 6650 2100 6650
Wire Wire Line
	2100 6550 2100 6650
Connection ~ 2100 6650
Wire Wire Line
	2100 6650 2200 6650
$Comp
L Regulator_Linear:XC6206PxxxMR U1
U 1 1 5F050A47
P 1150 1150
F 0 "U1" H 1150 1392 50  0000 C CNN
F 1 "XC6206P332MR" H 1150 1301 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1150 1375 50  0001 C CIN
F 3 "https://www.torexsemi.com/file/xc6206/XC6206.pdf" H 1150 1150 50  0001 C CNN
F 4 "C5446" H 1150 1150 50  0001 C CNN "LCSC"
	1    1150 1150
	1    0    0    -1  
$EndComp
Text Notes 1050 6100 0    50   ~ 0
Internal pull-up
$Comp
L power:VBUS #PWR012
U 1 1 5F008BB0
P 2750 2850
F 0 "#PWR012" H 2750 2700 50  0001 C CNN
F 1 "VBUS" H 2765 3023 50  0000 C CNN
F 2 "" H 2750 2850 50  0001 C CNN
F 3 "" H 2750 2850 50  0001 C CNN
	1    2750 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2950 3350 2950
Wire Wire Line
	3700 2950 3700 2850
$Comp
L power:+5V #PWR015
U 1 1 5F014A14
P 3700 2850
F 0 "#PWR015" H 3700 2700 50  0001 C CNN
F 1 "+5V" H 3715 3023 50  0000 C CNN
F 2 "" H 3700 2850 50  0001 C CNN
F 3 "" H 3700 2850 50  0001 C CNN
	1    3700 2850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 5F0C4D3B
P 750 1050
F 0 "#PWR01" H 750 900 50  0001 C CNN
F 1 "+5V" H 765 1223 50  0000 C CNN
F 2 "" H 750 1050 50  0001 C CNN
F 3 "" H 750 1050 50  0001 C CNN
	1    750  1050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR07
U 1 1 5F0C519F
P 1550 1050
F 0 "#PWR07" H 1550 900 50  0001 C CNN
F 1 "+3.3V" H 1565 1223 50  0000 C CNN
F 2 "" H 1550 1050 50  0001 C CNN
F 3 "" H 1550 1050 50  0001 C CNN
	1    1550 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1150 1550 1050
Wire Wire Line
	2800 1100 2800 1000
Connection ~ 2800 1100
Wire Wire Line
	2750 2850 2750 2950
Connection ~ 2750 2950
Wire Wire Line
	2750 2950 2900 2950
$Comp
L Device:R R1
U 1 1 5F13D7AC
P 2050 3450
F 0 "R1" V 2100 3600 50  0000 L CNN
F 1 "22R" V 2100 3150 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 1980 3450 50  0001 C CNN
F 3 "~" H 2050 3450 50  0001 C CNN
F 4 "C25092" H 2050 3450 50  0001 C CNN "LCSC"
	1    2050 3450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 5F13E4ED
P 2050 3650
F 0 "R2" V 2100 3800 50  0000 L CNN
F 1 "22R" V 2100 3350 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 1980 3650 50  0001 C CNN
F 3 "~" H 2050 3650 50  0001 C CNN
F 4 "C25092" H 2050 3650 50  0001 C CNN "LCSC"
	1    2050 3650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2650 3450 2200 3450
Wire Wire Line
	2650 3650 2200 3650
Wire Wire Line
	1700 3650 1900 3650
Wire Wire Line
	1700 3450 1900 3450
$Comp
L Device:R R3
U 1 1 5F17996F
P 2050 3750
F 0 "R3" V 2100 3900 50  0000 L CNN
F 1 "1.5k" V 2100 3450 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 1980 3750 50  0001 C CNN
F 3 "~" H 2050 3750 50  0001 C CNN
F 4 "C25867" H 2050 3750 50  0001 C CNN "LCSC"
	1    2050 3750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1900 3750 1700 3750
Connection ~ 1700 3750
$Comp
L power:+3.3V #PWR09
U 1 1 5F18590E
P 2450 2850
F 0 "#PWR09" H 2450 2700 50  0001 C CNN
F 1 "+3.3V" H 2465 3023 50  0000 C CNN
F 2 "" H 2450 2850 50  0001 C CNN
F 3 "" H 2450 2850 50  0001 C CNN
	1    2450 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 2850 2450 3750
Wire Wire Line
	2450 3750 2200 3750
$Comp
L power:+5V #PWR0101
U 1 1 5F1B0776
P 10200 800
F 0 "#PWR0101" H 10200 650 50  0001 C CNN
F 1 "+5V" H 10215 973 50  0000 C CNN
F 2 "" H 10200 800 50  0001 C CNN
F 3 "" H 10200 800 50  0001 C CNN
	1    10200 800 
	1    0    0    -1  
$EndComp
Wire Notes Line
	550  700  550  1950
Wire Notes Line
	550  1950 1800 1950
Wire Notes Line
	1800 1950 1800 700 
Wire Notes Line
	1800 700  550  700 
Text Notes 550  650  0    50   ~ 0
Power Regulation
Wire Notes Line
	2600 650  2600 1900
Wire Notes Line
	2600 1900 3850 1900
Wire Notes Line
	3850 1900 3850 650 
Wire Notes Line
	3850 650  2600 650 
Text Notes 2600 600  0    50   ~ 0
Decoupling
Wire Notes Line
	550  4950 3900 4950
Wire Notes Line
	3900 4950 3900 2450
Wire Notes Line
	3900 2450 550  2450
Wire Notes Line
	550  2450 550  4950
Text Notes 550  2400 0    50   ~ 0
USB C Connection
Wire Wire Line
	10450 900  10450 2800
Connection ~ 10450 2800
Wire Wire Line
	10550 2700 10200 2700
Wire Wire Line
	10200 2700 10200 800 
Wire Wire Line
	10550 3550 10450 3550
Wire Wire Line
	10450 3550 10450 5550
Text GLabel 10050 3650 0    50   Input ~ 0
RST
Text GLabel 10050 3750 0    50   Input ~ 0
BO0
Wire Wire Line
	10550 3650 10050 3650
Wire Wire Line
	10550 3750 10050 3750
Text GLabel 7050 2650 0    50   Input ~ 0
B2
Text GLabel 9900 2300 0    50   Input ~ 0
B2
Text Notes 9450 6350 0    50   ~ 0
NOTICE!  Headers are mostly but NOT\n100 % compatible with the blue/black pill.
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5F1B7E76
P 1850 2850
F 0 "#FLG0101" H 1850 2925 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 3023 50  0000 C CNN
F 2 "" H 1850 2850 50  0001 C CNN
F 3 "~" H 1850 2850 50  0001 C CNN
	1    1850 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2850 1850 2950
Connection ~ 1850 2950
Wire Wire Line
	1850 2950 2750 2950
NoConn ~ 1600 4050
NoConn ~ 1600 4150
NoConn ~ 3800 4450
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5F1F0DFA
P 1150 1550
F 0 "#FLG0102" H 1150 1625 50  0001 C CNN
F 1 "PWR_FLAG" V 1150 1678 50  0000 L CNN
F 2 "" H 1150 1550 50  0001 C CNN
F 3 "~" H 1150 1550 50  0001 C CNN
	1    1150 1550
	0    1    1    0   
$EndComp
Connection ~ 1150 1550
Wire Wire Line
	1150 1550 1150 1450
NoConn ~ 3150 900 
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 5F2349A8
P 3350 2850
F 0 "#FLG0103" H 3350 2925 50  0001 C CNN
F 1 "PWR_FLAG" H 3350 3023 50  0000 C CNN
F 2 "" H 3350 2850 50  0001 C CNN
F 3 "~" H 3350 2850 50  0001 C CNN
	1    3350 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 2850 3350 2950
Connection ~ 3350 2950
Wire Wire Line
	3350 2950 3700 2950
$EndSCHEMATC
