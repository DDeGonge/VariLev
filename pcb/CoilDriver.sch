EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
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
L power:GND #PWR0104
U 1 1 61024B80
P 3350 4150
AR Path="/60FFED8C/61024B80" Ref="#PWR0104"  Part="1" 
AR Path="/6110159B/61024B80" Ref="#PWR0128"  Part="1" 
F 0 "#PWR0128" H 3350 3900 50  0001 C CNN
F 1 "GND" H 3355 3977 50  0000 C CNN
F 2 "" H 3350 4150 50  0001 C CNN
F 3 "" H 3350 4150 50  0001 C CNN
	1    3350 4150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J5
U 1 1 61027739
P 4300 5200
AR Path="/60FFED8C/61027739" Ref="J5"  Part="1" 
AR Path="/6110159B/61027739" Ref="J8"  Part="1" 
F 0 "J5" V 4172 5280 50  0000 L CNN
F 1 "Conn_01x02" V 4263 5280 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 4300 5200 50  0001 C CNN
F 3 "~" H 4300 5200 50  0001 C CNN
	1    4300 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 4750 4250 4900
Wire Wire Line
	4250 4900 4200 4900
Wire Wire Line
	4200 4900 4200 5000
Wire Wire Line
	4300 5000 4300 4900
Wire Wire Line
	4300 4900 4350 4900
Wire Wire Line
	4350 4900 4350 4750
Wire Wire Line
	4450 4750 4450 4900
Wire Wire Line
	4450 4900 4500 4900
Wire Wire Line
	4500 4900 4500 5000
Wire Wire Line
	4550 4750 4550 4900
Wire Wire Line
	4550 4900 4600 4900
Wire Wire Line
	4600 4900 4600 5000
Wire Wire Line
	3350 4150 3350 4050
Wire Wire Line
	3350 4050 3550 4050
Wire Wire Line
	3550 3850 3350 3850
Wire Wire Line
	3350 3850 3350 4050
Connection ~ 3350 4050
Wire Wire Line
	4350 2350 4350 2550
Wire Wire Line
	4450 2550 4450 2350
Wire Wire Line
	4550 2350 4550 2550
$Comp
L power:+3V3 #PWR0116
U 1 1 6102D010
P 5750 3500
AR Path="/60FFED8C/6102D010" Ref="#PWR0116"  Part="1" 
AR Path="/6110159B/6102D010" Ref="#PWR0129"  Part="1" 
F 0 "#PWR0129" H 5750 3350 50  0001 C CNN
F 1 "+3V3" H 5765 3673 50  0000 C CNN
F 2 "" H 5750 3500 50  0001 C CNN
F 3 "" H 5750 3500 50  0001 C CNN
	1    5750 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3750 5750 3750
Wire Wire Line
	5750 3750 5750 3500
Wire Wire Line
	2900 3750 2900 3800
Wire Wire Line
	2900 3950 3550 3950
Wire Wire Line
	3350 3750 3550 3750
$Comp
L power:GND #PWR0119
U 1 1 610306B5
P 5500 4550
AR Path="/60FFED8C/610306B5" Ref="#PWR0119"  Part="1" 
AR Path="/6110159B/610306B5" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0130" H 5500 4300 50  0001 C CNN
F 1 "GND" H 5505 4377 50  0000 C CNN
F 2 "" H 5500 4550 50  0001 C CNN
F 3 "" H 5500 4550 50  0001 C CNN
	1    5500 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 61031BFA
P 3950 2500
AR Path="/60FFED8C/61031BFA" Ref="#PWR0124"  Part="1" 
AR Path="/6110159B/61031BFA" Ref="#PWR0131"  Part="1" 
F 0 "#PWR0131" H 3950 2250 50  0001 C CNN
F 1 "GND" H 3955 2327 50  0000 C CNN
F 2 "" H 3950 2500 50  0001 C CNN
F 3 "" H 3950 2500 50  0001 C CNN
	1    3950 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2500 3950 2400
Wire Wire Line
	3950 2400 4250 2400
Wire Wire Line
	4250 2400 4250 2550
$Comp
L Device:R R4
U 1 1 61032577
P 5500 4250
AR Path="/60FFED8C/61032577" Ref="R4"  Part="1" 
AR Path="/6110159B/61032577" Ref="R8"  Part="1" 
F 0 "R4" H 5570 4296 50  0000 L CNN
F 1 "75k" H 5570 4205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5430 4250 50  0001 C CNN
F 3 "~" H 5500 4250 50  0001 C CNN
	1    5500 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3850 5500 3850
$Comp
L Device:R R3
U 1 1 61035F55
P 5250 2200
AR Path="/60FFED8C/61035F55" Ref="R3"  Part="1" 
AR Path="/6110159B/61035F55" Ref="R7"  Part="1" 
F 0 "R3" V 5457 2200 50  0000 C CNN
F 1 "10k" V 5366 2200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5180 2200 50  0001 C CNN
F 3 "~" H 5250 2200 50  0001 C CNN
	1    5250 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4650 2550 4650 2350
Wire Wire Line
	4650 2350 4750 2350
$Comp
L Device:C C19
U 1 1 61036AD8
P 5000 2350
AR Path="/60FFED8C/61036AD8" Ref="C19"  Part="1" 
AR Path="/6110159B/61036AD8" Ref="C22"  Part="1" 
F 0 "C19" H 5115 2396 50  0000 L CNN
F 1 "22nF" H 5115 2305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5038 2200 50  0001 C CNN
F 3 "~" H 5000 2350 50  0001 C CNN
	1    5000 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 2200 5550 2200
$Comp
L SamacSys_Parts:TC78H660FTG,EL IC3
U 1 1 610228BD
P 3550 3750
AR Path="/60FFED8C/610228BD" Ref="IC3"  Part="1" 
AR Path="/6110159B/610228BD" Ref="IC4"  Part="1" 
F 0 "IC3" H 5394 3896 50  0000 L CNN
F 1 "TC78H660FTG,EL" H 5394 3805 50  0000 L CNN
F 2 "QFN50P300X300X90-17N" H 5200 4750 50  0001 L CNN
F 3 "https://toshiba.semicon-storage.com/eu/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TC78H660FTG.html" H 5200 4650 50  0001 L CNN
F 4 "Motor / Motion / Ignition Controllers & Drivers Dual H Brdge Drvr IC" H 5200 4550 50  0001 L CNN "Description"
F 5 "0.9" H 5200 4450 50  0001 L CNN "Height"
F 6 "757-TC78H660FTGEL" H 5200 4350 50  0001 L CNN "Mouser Part Number"
F 7 "https://www.mouser.co.uk/ProductDetail/Toshiba/TC78H660FTGEL?qs=GBLSl2Akirut0oT06igwhw%3D%3D" H 5200 4250 50  0001 L CNN "Mouser Price/Stock"
F 8 "Toshiba" H 5200 4150 50  0001 L CNN "Manufacturer_Name"
F 9 "TC78H660FTG,EL" H 5200 4050 50  0001 L CNN "Manufacturer_Part_Number"
	1    3550 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2200 4650 2350
Connection ~ 4650 2350
Wire Wire Line
	4750 2350 4750 2200
Wire Wire Line
	4750 2200 5000 2200
$Comp
L power:GND #PWR0125
U 1 1 6104514C
P 5000 2550
AR Path="/60FFED8C/6104514C" Ref="#PWR0125"  Part="1" 
AR Path="/6110159B/6104514C" Ref="#PWR0132"  Part="1" 
F 0 "#PWR0132" H 5000 2300 50  0001 C CNN
F 1 "GND" H 5005 2377 50  0000 C CNN
F 2 "" H 5000 2550 50  0001 C CNN
F 3 "" H 5000 2550 50  0001 C CNN
	1    5000 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2500 5000 2550
Wire Wire Line
	5000 2200 5100 2200
Connection ~ 5000 2200
$Comp
L Device:R R5
U 1 1 61047B1F
P 5950 4000
AR Path="/60FFED8C/61047B1F" Ref="R5"  Part="1" 
AR Path="/6110159B/61047B1F" Ref="R9"  Part="1" 
F 0 "R5" H 6020 4046 50  0000 L CNN
F 1 "10k" H 6020 3955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5880 4000 50  0001 C CNN
F 3 "~" H 5950 4000 50  0001 C CNN
	1    5950 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4050 5400 4050
Wire Wire Line
	5400 4050 5400 4450
Wire Wire Line
	5400 4450 5500 4450
Wire Wire Line
	5500 4450 5500 4550
Wire Wire Line
	5500 4400 5500 4450
Connection ~ 5500 4450
Wire Wire Line
	5500 4100 5500 3850
$Comp
L Device:R R6
U 1 1 6104D7C6
P 5950 4400
AR Path="/60FFED8C/6104D7C6" Ref="R6"  Part="1" 
AR Path="/6110159B/6104D7C6" Ref="R10"  Part="1" 
F 0 "R6" H 6020 4446 50  0000 L CNN
F 1 "10k" H 6020 4355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5880 4400 50  0001 C CNN
F 3 "~" H 5950 4400 50  0001 C CNN
	1    5950 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 4550 5950 4600
Wire Wire Line
	5950 4600 5700 4600
Wire Wire Line
	5700 4600 5700 4450
Wire Wire Line
	5700 4450 5500 4450
Wire Wire Line
	5950 4150 5950 4200
Wire Wire Line
	5950 4200 5750 4200
Wire Wire Line
	5750 4200 5750 3950
Wire Wire Line
	5750 3950 5350 3950
Connection ~ 5950 4200
Wire Wire Line
	5950 4200 5950 4250
Wire Wire Line
	5750 3750 5950 3750
Wire Wire Line
	5950 3750 5950 3850
Connection ~ 5750 3750
$Comp
L Device:C C20
U 1 1 61051878
P 6350 4400
AR Path="/60FFED8C/61051878" Ref="C20"  Part="1" 
AR Path="/6110159B/61051878" Ref="C23"  Part="1" 
F 0 "C20" H 6465 4446 50  0000 L CNN
F 1 "100nF" H 6465 4355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6388 4250 50  0001 C CNN
F 3 "~" H 6350 4400 50  0001 C CNN
	1    6350 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 4550 6350 4600
Wire Wire Line
	6350 4600 5950 4600
Connection ~ 5950 4600
Wire Wire Line
	5950 4200 6350 4200
Wire Wire Line
	6350 4200 6350 4250
$Comp
L Device:C C?
U 1 1 61057EC2
P 2700 4000
AR Path="/61057EC2" Ref="C?"  Part="1" 
AR Path="/60FFED8C/61057EC2" Ref="C18"  Part="1" 
AR Path="/6110159B/61057EC2" Ref="C21"  Part="1" 
F 0 "C18" H 2815 4046 50  0000 L CNN
F 1 "47uF" H 2815 3955 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 2738 3850 50  0001 C CNN
F 3 "~" H 2700 4000 50  0001 C CNN
	1    2700 4000
	-1   0    0    1   
$EndComp
Wire Wire Line
	2700 3850 2700 3800
Wire Wire Line
	2700 3800 2900 3800
Connection ~ 2900 3800
Wire Wire Line
	2900 3800 2900 3950
Wire Wire Line
	2700 4150 2700 4200
Wire Wire Line
	2700 4200 2900 4200
Wire Wire Line
	2900 4200 2900 4050
Wire Wire Line
	2900 4050 3350 4050
Text HLabel 4350 2350 1    50   Input ~ 0
IN2B
Text HLabel 4450 2350 1    50   Input ~ 0
IN1B
Text HLabel 4550 2350 1    50   Input ~ 0
IN2A
Text HLabel 4650 2200 1    50   Output ~ 0
ERR
Text HLabel 3350 3750 0    50   Input ~ 0
IN1A
Text HLabel 5550 2200 2    50   Input ~ 0
MODE
$Comp
L Connector_Generic:Conn_01x02 J7
U 1 1 61029251
P 4600 5200
AR Path="/60FFED8C/61029251" Ref="J7"  Part="1" 
AR Path="/6110159B/61029251" Ref="J9"  Part="1" 
F 0 "J7" V 4472 5280 50  0000 L CNN
F 1 "Conn_01x02" V 4563 5280 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 4600 5200 50  0001 C CNN
F 3 "~" H 4600 5200 50  0001 C CNN
	1    4600 5200
	0    1    1    0   
$EndComp
$Comp
L power:+VDC #PWR0102
U 1 1 615F5340
P 2900 3750
AR Path="/60FFED8C/615F5340" Ref="#PWR0102"  Part="1" 
AR Path="/6110159B/615F5340" Ref="#PWR0126"  Part="1" 
F 0 "#PWR0126" H 2900 3650 50  0001 C CNN
F 1 "+VDC" H 2900 4025 50  0000 C CNN
F 2 "" H 2900 3750 50  0001 C CNN
F 3 "" H 2900 3750 50  0001 C CNN
	1    2900 3750
	1    0    0    -1  
$EndComp
$EndSCHEMATC
