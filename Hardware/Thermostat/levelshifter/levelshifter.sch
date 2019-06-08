EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
Title "Levelshifter"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L TXB0104D U1
U 1 1 5AD5FAA6
P 5700 3500
F 0 "U1" H 5425 4155 50  0000 C CNN
F 1 "TXB0104D" H 6050 4150 50  0000 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 5750 2650 50  0001 C CNN
F 3 "" H 5810 3595 50  0001 C CNN
	1    5700 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5AD5FC6C
P 5700 4600
F 0 "#PWR01" H 5700 4350 50  0001 C CNN
F 1 "GND" H 5700 4450 50  0000 C CNN
F 2 "" H 5700 4600 50  0001 C CNN
F 3 "" H 5700 4600 50  0001 C CNN
	1    5700 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4200 5700 4600
$Comp
L +3.3V #PWR02
U 1 1 5AD5FCB8
P 5600 2350
F 0 "#PWR02" H 5600 2200 50  0001 C CNN
F 1 "+3.3V" H 5600 2490 50  0000 C CNN
F 2 "" H 5600 2350 50  0001 C CNN
F 3 "" H 5600 2350 50  0001 C CNN
	1    5600 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 2350 5600 2800
$Comp
L +5V #PWR03
U 1 1 5AD5FCEB
P 5800 2350
F 0 "#PWR03" H 5800 2200 50  0001 C CNN
F 1 "+5V" H 5800 2490 50  0000 C CNN
F 2 "" H 5800 2350 50  0001 C CNN
F 3 "" H 5800 2350 50  0001 C CNN
	1    5800 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 2350 5800 2800
$Comp
L C C1
U 1 1 5AD5FDB9
P 7100 3300
F 0 "C1" H 7125 3400 50  0000 L CNN
F 1 "100nF" H 7125 3200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7138 3150 50  0001 C CNN
F 3 "" H 7100 3300 50  0001 C CNN
	1    7100 3300
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR04
U 1 1 5AD5FE05
P 7100 3050
F 0 "#PWR04" H 7100 2900 50  0001 C CNN
F 1 "+5V" H 7100 3190 50  0000 C CNN
F 2 "" H 7100 3050 50  0001 C CNN
F 3 "" H 7100 3050 50  0001 C CNN
	1    7100 3050
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5AD5FE19
P 7450 3300
F 0 "C2" H 7475 3400 50  0000 L CNN
F 1 "100nF" H 7475 3200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7488 3150 50  0001 C CNN
F 3 "" H 7450 3300 50  0001 C CNN
	1    7450 3300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 5AD5FE69
P 7450 3050
F 0 "#PWR05" H 7450 2900 50  0001 C CNN
F 1 "+3.3V" H 7450 3190 50  0000 C CNN
F 2 "" H 7450 3050 50  0001 C CNN
F 3 "" H 7450 3050 50  0001 C CNN
	1    7450 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3150 7450 3050
Wire Wire Line
	7100 3050 7100 3150
$Comp
L GND #PWR06
U 1 1 5AD5FEB3
P 7100 3600
F 0 "#PWR06" H 7100 3350 50  0001 C CNN
F 1 "GND" H 7100 3450 50  0000 C CNN
F 2 "" H 7100 3600 50  0001 C CNN
F 3 "" H 7100 3600 50  0001 C CNN
	1    7100 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5AD5FECA
P 7450 3600
F 0 "#PWR07" H 7450 3350 50  0001 C CNN
F 1 "GND" H 7450 3450 50  0000 C CNN
F 2 "" H 7450 3600 50  0001 C CNN
F 3 "" H 7450 3600 50  0001 C CNN
	1    7450 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 3600 7450 3450
Wire Wire Line
	7100 3600 7100 3450
$Comp
L Conn_01x04 J3
U 1 1 5AD5FEFA
P 6550 3400
F 0 "J3" H 6550 3600 50  0000 C CNN
F 1 "Conn_01x04" H 6550 3100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 6550 3400 50  0001 C CNN
F 3 "" H 6550 3400 50  0001 C CNN
	1    6550 3400
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x04 J2
U 1 1 5AD5FF46
P 4400 3400
F 0 "J2" H 4400 3600 50  0000 C CNN
F 1 "Conn_01x04" H 4400 3100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4400 3400 50  0001 C CNN
F 3 "" H 4400 3400 50  0001 C CNN
	1    4400 3400
	-1   0    0    -1  
$EndComp
$Comp
L Conn_01x03 J1
U 1 1 5AD5FFCA
P 8600 3350
F 0 "J1" H 8600 3550 50  0000 C CNN
F 1 "Conn_01x03" H 8600 3150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 8600 3350 50  0001 C CNN
F 3 "" H 8600 3350 50  0001 C CNN
	1    8600 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5AD60147
P 8100 3350
F 0 "#PWR08" H 8100 3100 50  0001 C CNN
F 1 "GND" H 8100 3200 50  0000 C CNN
F 2 "" H 8100 3350 50  0001 C CNN
F 3 "" H 8100 3350 50  0001 C CNN
	1    8100 3350
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR09
U 1 1 5AD6016A
P 8150 3150
F 0 "#PWR09" H 8150 3000 50  0001 C CNN
F 1 "+3.3V" H 8150 3290 50  0000 C CNN
F 2 "" H 8150 3150 50  0001 C CNN
F 3 "" H 8150 3150 50  0001 C CNN
	1    8150 3150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR010
U 1 1 5AD60187
P 8200 3700
F 0 "#PWR010" H 8200 3550 50  0001 C CNN
F 1 "+5V" H 8200 3840 50  0000 C CNN
F 2 "" H 8200 3700 50  0001 C CNN
F 3 "" H 8200 3700 50  0001 C CNN
	1    8200 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	8400 3450 8200 3450
Wire Wire Line
	8200 3450 8200 3700
Wire Wire Line
	8400 3250 8150 3250
Wire Wire Line
	8150 3250 8150 3150
Wire Wire Line
	8400 3350 8100 3350
Wire Wire Line
	5300 3200 4600 3200
Wire Wire Line
	4600 3200 4600 3300
Wire Wire Line
	4600 3400 5300 3400
Wire Wire Line
	5300 3600 4950 3600
Wire Wire Line
	4950 3600 4950 3500
Wire Wire Line
	4950 3500 4600 3500
Wire Wire Line
	4600 3600 4750 3600
Wire Wire Line
	4750 3600 4750 3800
Wire Wire Line
	4750 3800 5300 3800
Wire Wire Line
	6100 3200 6350 3200
Wire Wire Line
	6350 3200 6350 3300
Wire Wire Line
	6100 3400 6350 3400
Wire Wire Line
	6350 3500 6150 3500
Wire Wire Line
	6150 3500 6150 3600
Wire Wire Line
	6150 3600 6100 3600
Wire Wire Line
	6100 3800 6250 3800
Wire Wire Line
	6250 3800 6250 3600
Wire Wire Line
	6250 3600 6350 3600
$Sheet
S 600  600  1350 600 
U 5AD607E7
F0 "spannungsversorungen.sch" 60
F1 "spannungsversorungen.sch" 60
$EndSheet
$Comp
L +3.3V #PWR011
U 1 1 5AD609BE
P 5050 2900
F 0 "#PWR011" H 5050 2750 50  0001 C CNN
F 1 "+3.3V" H 5050 3040 50  0000 C CNN
F 2 "" H 5050 2900 50  0001 C CNN
F 3 "" H 5050 2900 50  0001 C CNN
	1    5050 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 2900 5050 3000
Wire Wire Line
	5050 3000 5300 3000
$EndSCHEMATC