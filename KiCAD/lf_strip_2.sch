EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
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
LIBS:w_analog
LIBS:w_connectors
LIBS:w_device
LIBS:w_logic
LIBS:w_memory
LIBS:w_microcontrollers
LIBS:w_opto
LIBS:w_relay
LIBS:w_rtx
LIBS:w_transistor
LIBS:w_freaquency
LIBS:lf_strip_2-cache
EELAYER 25 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 1 1
Title "Linefollower Sensorstrip"
Date "2015-04-20"
Rev ""
Comp ""
Comment1 "Fredrik Åkerlund"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ITR8307/TR8 O1
U 1 1 54F4D7CC
P 1850 2150
F 0 "O1" H 2050 1750 60  0000 C CNN
F 1 "ITR8307/TR8" V 1650 2250 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 1850 2150 60  0001 C CNN
F 3 "" H 1850 2150 60  0000 C CNN
	1    1850 2150
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 54F4F0B2
P 1950 1250
F 0 "R1" V 1850 1250 50  0000 C CNN
F 1 "10k" V 1957 1251 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 1880 1250 30  0001 C CNN
F 3 "" H 1950 1250 30  0000 C CNN
	1    1950 1250
	1    0    0    -1  
$EndComp
Text GLabel 3200 3400 0    60   Input ~ 0
ADC10
Text GLabel 3850 3400 0    60   Input ~ 0
ADC11
Text GLabel 4500 3400 0    60   Input ~ 0
ADC12
Text GLabel 5150 3400 0    60   Input ~ 0
ADC13
Text GLabel 1900 1500 0    60   Input ~ 0
ADC0
Text GLabel 2550 3400 0    60   Input ~ 0
ADC9
Text GLabel 2550 1500 0    60   Input ~ 0
ADC1
Text GLabel 3200 1500 0    60   Input ~ 0
ADC2
Text GLabel 3850 1500 0    60   Input ~ 0
ADC3
Text GLabel 4500 1500 0    60   Input ~ 0
ADC4
Text GLabel 5150 1500 0    60   Input ~ 0
ADC5
Text GLabel 5800 1500 0    60   Input ~ 0
ADC6
Text GLabel 6450 1500 0    60   Input ~ 0
ADC7
Text GLabel 1900 3400 0    60   Input ~ 0
ADC8
Text GLabel 2100 1000 3    60   Input ~ 0
VDD50
$Comp
L R R2
U 1 1 54F51261
P 2200 2000
F 0 "R2" V 2280 2000 50  0000 C CNN
F 1 "150" V 2207 2001 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 2130 2000 30  0001 C CNN
F 3 "" H 2200 2000 30  0000 C CNN
	1    2200 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 1400 1950 1550
Wire Wire Line
	2200 2150 2200 2450
Wire Wire Line
	2200 2450 2050 2450
Wire Wire Line
	1950 1500 1900 1500
$Comp
L ITR8307/TR8 O2
U 1 1 54F536B8
P 2500 2150
F 0 "O2" H 2700 1750 60  0000 C CNN
F 1 "ITR8307/TR8" V 2300 2250 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 2500 2150 60  0001 C CNN
F 3 "" H 2500 2150 60  0000 C CNN
	1    2500 2150
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 54F536BE
P 2600 1250
F 0 "R3" V 2500 1250 50  0000 C CNN
F 1 "10k" V 2607 1251 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 2530 1250 30  0001 C CNN
F 3 "" H 2600 1250 30  0000 C CNN
	1    2600 1250
	1    0    0    -1  
$EndComp
Text GLabel 2750 1000 3    60   Input ~ 0
VDD50
$Comp
L R R4
U 1 1 54F536C7
P 2850 2000
F 0 "R4" V 2930 2000 50  0000 C CNN
F 1 "150" V 2857 2001 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 2780 2000 30  0001 C CNN
F 3 "" H 2850 2000 30  0000 C CNN
	1    2850 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1400 2600 1550
Wire Wire Line
	2850 2150 2850 2450
Wire Wire Line
	2850 2450 2700 2450
Wire Wire Line
	2600 1500 2550 1500
$Comp
L ITR8307/TR8 O3
U 1 1 54F53894
P 3150 2150
F 0 "O3" H 3350 1750 60  0000 C CNN
F 1 "ITR8307/TR8" V 2950 2250 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 3150 2150 60  0001 C CNN
F 3 "" H 3150 2150 60  0000 C CNN
	1    3150 2150
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 54F5389A
P 3250 1250
F 0 "R5" V 3150 1250 50  0000 C CNN
F 1 "10k" V 3257 1251 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 3180 1250 30  0001 C CNN
F 3 "" H 3250 1250 30  0000 C CNN
	1    3250 1250
	1    0    0    -1  
$EndComp
Text GLabel 3400 1000 3    60   Input ~ 0
VDD50
$Comp
L R R6
U 1 1 54F538A3
P 3500 2000
F 0 "R6" V 3580 2000 50  0000 C CNN
F 1 "150" V 3507 2001 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 3430 2000 30  0001 C CNN
F 3 "" H 3500 2000 30  0000 C CNN
	1    3500 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 1400 3250 1550
Wire Wire Line
	3500 2150 3500 2450
Wire Wire Line
	3500 2450 3350 2450
Wire Wire Line
	3250 1500 3200 1500
$Comp
L ITR8307/TR8 O4
U 1 1 54F538B0
P 3800 2150
F 0 "O4" H 4000 1750 60  0000 C CNN
F 1 "ITR8307/TR8" V 3600 2250 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 3800 2150 60  0001 C CNN
F 3 "" H 3800 2150 60  0000 C CNN
	1    3800 2150
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 54F538B6
P 3900 1250
F 0 "R7" V 3800 1250 50  0000 C CNN
F 1 "10k" V 3907 1251 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 3830 1250 30  0001 C CNN
F 3 "" H 3900 1250 30  0000 C CNN
	1    3900 1250
	1    0    0    -1  
$EndComp
Text GLabel 4050 1000 3    60   Input ~ 0
VDD50
$Comp
L R R8
U 1 1 54F538BE
P 4150 2000
F 0 "R8" V 4230 2000 50  0000 C CNN
F 1 "150" V 4157 2001 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 4080 2000 30  0001 C CNN
F 3 "" H 4150 2000 30  0000 C CNN
	1    4150 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1400 3900 1550
Wire Wire Line
	4150 2150 4150 2450
Wire Wire Line
	4150 2450 4000 2450
Wire Wire Line
	3900 1500 3850 1500
$Comp
L ITR8307/TR8 O5
U 1 1 54F53B95
P 4450 2150
F 0 "O5" H 4650 1750 60  0000 C CNN
F 1 "ITR8307/TR8" V 4250 2250 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 4450 2150 60  0001 C CNN
F 3 "" H 4450 2150 60  0000 C CNN
	1    4450 2150
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 54F53B9B
P 4550 1250
F 0 "R9" V 4450 1250 50  0000 C CNN
F 1 "10k" V 4557 1251 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 4480 1250 30  0001 C CNN
F 3 "" H 4550 1250 30  0000 C CNN
	1    4550 1250
	1    0    0    -1  
$EndComp
Text GLabel 4700 1000 3    60   Input ~ 0
VDD50
$Comp
L R R10
U 1 1 54F53BA7
P 4800 2000
F 0 "R10" V 4880 2000 50  0000 C CNN
F 1 "150" V 4807 2001 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 4730 2000 30  0001 C CNN
F 3 "" H 4800 2000 30  0000 C CNN
	1    4800 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1400 4550 1550
Wire Wire Line
	4800 1000 4550 1000
Wire Wire Line
	4800 1000 4800 1850
Wire Wire Line
	4800 2150 4800 2450
Wire Wire Line
	4800 2450 4650 2450
Wire Wire Line
	4550 1500 4500 1500
$Comp
L ITR8307/TR8 O6
U 1 1 54F53BB4
P 5100 2150
F 0 "O6" H 5300 1750 60  0000 C CNN
F 1 "ITR8307/TR8" V 4900 2250 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 5100 2150 60  0001 C CNN
F 3 "" H 5100 2150 60  0000 C CNN
	1    5100 2150
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 54F53BBA
P 5200 1250
F 0 "R11" V 5100 1250 50  0000 C CNN
F 1 "10k" V 5207 1251 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 5130 1250 30  0001 C CNN
F 3 "" H 5200 1250 30  0000 C CNN
	1    5200 1250
	1    0    0    -1  
$EndComp
Text GLabel 5350 1000 3    60   Input ~ 0
VDD50
$Comp
L R R12
U 1 1 54F53BC2
P 5450 2000
F 0 "R12" V 5530 2000 50  0000 C CNN
F 1 "150" V 5457 2001 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 5380 2000 30  0001 C CNN
F 3 "" H 5450 2000 30  0000 C CNN
	1    5450 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1400 5200 1550
Wire Wire Line
	5450 1000 5200 1000
Wire Wire Line
	5450 1000 5450 1850
Wire Wire Line
	5450 2150 5450 2450
Wire Wire Line
	5450 2450 5300 2450
Wire Wire Line
	5200 1500 5150 1500
$Comp
L ITR8307/TR8 O7
U 1 1 54F53BCF
P 5750 2150
F 0 "O7" H 5950 1750 60  0000 C CNN
F 1 "ITR8307/TR8" V 5550 2250 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 5750 2150 60  0001 C CNN
F 3 "" H 5750 2150 60  0000 C CNN
	1    5750 2150
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 54F53BD5
P 5850 1250
F 0 "R13" V 5750 1250 50  0000 C CNN
F 1 "10k" V 5857 1251 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 5780 1250 30  0001 C CNN
F 3 "" H 5850 1250 30  0000 C CNN
	1    5850 1250
	1    0    0    -1  
$EndComp
Text GLabel 6000 1000 3    60   Input ~ 0
VDD50
$Comp
L R R14
U 1 1 54F53BDD
P 6100 2000
F 0 "R14" V 6180 2000 50  0000 C CNN
F 1 "150" V 6107 2001 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 6030 2000 30  0001 C CNN
F 3 "" H 6100 2000 30  0000 C CNN
	1    6100 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 1400 5850 1550
Wire Wire Line
	6100 1000 5850 1000
Wire Wire Line
	6100 1000 6100 1850
Wire Wire Line
	6100 2150 6100 2450
Wire Wire Line
	6100 2450 5950 2450
Wire Wire Line
	5850 1500 5800 1500
$Comp
L R R15
U 1 1 54F53BF0
P 6500 1250
F 0 "R15" V 6400 1250 50  0000 C CNN
F 1 "10k" V 6507 1251 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 6430 1250 30  0001 C CNN
F 3 "" H 6500 1250 30  0000 C CNN
	1    6500 1250
	1    0    0    -1  
$EndComp
Text GLabel 6650 1000 3    60   Input ~ 0
VDD50
$Comp
L R R16
U 1 1 54F53BF8
P 6750 2000
F 0 "R16" V 6830 2000 50  0000 C CNN
F 1 "150" V 6757 2001 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 6680 2000 30  0001 C CNN
F 3 "" H 6750 2000 30  0000 C CNN
	1    6750 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 1400 6500 1550
Wire Wire Line
	6750 1000 6500 1000
Wire Wire Line
	6750 1000 6750 1850
Wire Wire Line
	6750 2150 6750 2450
Wire Wire Line
	6750 2450 6600 2450
Wire Wire Line
	6500 1500 6450 1500
$Comp
L ITR8307/TR8 O9
U 1 1 54F55215
P 1850 4050
F 0 "O9" H 2050 3650 60  0000 C CNN
F 1 "ITR8307/TR8" V 1650 4150 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 1850 4050 60  0001 C CNN
F 3 "" H 1850 4050 60  0000 C CNN
	1    1850 4050
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 54F5521B
P 1950 3150
F 0 "R17" V 1850 3150 50  0000 C CNN
F 1 "10k" V 1957 3151 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 1880 3150 30  0001 C CNN
F 3 "" H 1950 3150 30  0000 C CNN
	1    1950 3150
	1    0    0    -1  
$EndComp
Text GLabel 2100 2900 3    60   Input ~ 0
VDD50
$Comp
L R R18
U 1 1 54F5522B
P 2200 3900
F 0 "R18" V 2280 3900 50  0000 C CNN
F 1 "150" V 2207 3901 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 2130 3900 30  0001 C CNN
F 3 "" H 2200 3900 30  0000 C CNN
	1    2200 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3300 1950 3450
Wire Wire Line
	2200 2900 1950 2900
Wire Wire Line
	2200 2900 2200 3750
Wire Wire Line
	2200 4050 2200 4350
Wire Wire Line
	2200 4350 2050 4350
Wire Wire Line
	1950 3400 1900 3400
$Comp
L ITR8307/TR8 O10
U 1 1 54F55238
P 2500 4050
F 0 "O10" H 2700 3650 60  0000 C CNN
F 1 "ITR8307/TR8" V 2300 4150 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 2500 4050 60  0001 C CNN
F 3 "" H 2500 4050 60  0000 C CNN
	1    2500 4050
	1    0    0    -1  
$EndComp
$Comp
L R R19
U 1 1 54F5523E
P 2600 3150
F 0 "R19" V 2500 3150 50  0000 C CNN
F 1 "10k" V 2607 3151 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 2530 3150 30  0001 C CNN
F 3 "" H 2600 3150 30  0000 C CNN
	1    2600 3150
	1    0    0    -1  
$EndComp
Text GLabel 2750 2900 3    60   Input ~ 0
VDD50
$Comp
L R R20
U 1 1 54F55246
P 2850 3900
F 0 "R20" V 2930 3900 50  0000 C CNN
F 1 "150" V 2857 3901 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 2780 3900 30  0001 C CNN
F 3 "" H 2850 3900 30  0000 C CNN
	1    2850 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3300 2600 3450
Wire Wire Line
	2850 2900 2600 2900
Wire Wire Line
	2850 2900 2850 3750
Wire Wire Line
	2850 4050 2850 4350
Wire Wire Line
	2850 4350 2700 4350
Wire Wire Line
	2600 3400 2550 3400
$Comp
L ITR8307/TR8 O11
U 1 1 54F55253
P 3150 4050
F 0 "O11" H 3350 3650 60  0000 C CNN
F 1 "ITR8307/TR8" V 2950 4150 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 3150 4050 60  0001 C CNN
F 3 "" H 3150 4050 60  0000 C CNN
	1    3150 4050
	1    0    0    -1  
$EndComp
$Comp
L R R21
U 1 1 54F55259
P 3250 3150
F 0 "R21" V 3150 3150 50  0000 C CNN
F 1 "10k" V 3257 3151 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 3180 3150 30  0001 C CNN
F 3 "" H 3250 3150 30  0000 C CNN
	1    3250 3150
	1    0    0    -1  
$EndComp
Text GLabel 3400 2900 3    60   Input ~ 0
VDD50
$Comp
L R R22
U 1 1 54F55261
P 3500 3900
F 0 "R22" V 3580 3900 50  0000 C CNN
F 1 "150" V 3507 3901 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 3430 3900 30  0001 C CNN
F 3 "" H 3500 3900 30  0000 C CNN
	1    3500 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 3300 3250 3450
Wire Wire Line
	3500 2900 3250 2900
Wire Wire Line
	3500 2900 3500 3750
Wire Wire Line
	3500 4050 3500 4350
Wire Wire Line
	3500 4350 3350 4350
Wire Wire Line
	3250 3400 3200 3400
$Comp
L ITR8307/TR8 O12
U 1 1 54F5526E
P 3800 4050
F 0 "O12" H 4000 3650 60  0000 C CNN
F 1 "ITR8307/TR8" V 3600 4150 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 3800 4050 60  0001 C CNN
F 3 "" H 3800 4050 60  0000 C CNN
	1    3800 4050
	1    0    0    -1  
$EndComp
$Comp
L R R23
U 1 1 54F55274
P 3900 3150
F 0 "R23" V 3800 3150 50  0000 C CNN
F 1 "10k" V 3907 3151 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 3830 3150 30  0001 C CNN
F 3 "" H 3900 3150 30  0000 C CNN
	1    3900 3150
	1    0    0    -1  
$EndComp
Text GLabel 4050 2900 3    60   Input ~ 0
VDD50
$Comp
L R R24
U 1 1 54F5527C
P 4150 3900
F 0 "R24" V 4230 3900 50  0000 C CNN
F 1 "150" V 4157 3901 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 4080 3900 30  0001 C CNN
F 3 "" H 4150 3900 30  0000 C CNN
	1    4150 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3300 3900 3450
Wire Wire Line
	4150 2900 3900 2900
Wire Wire Line
	4150 2900 4150 3750
Wire Wire Line
	4150 4050 4150 4350
Wire Wire Line
	4150 4350 4000 4350
Wire Wire Line
	3900 3400 3850 3400
$Comp
L ITR8307/TR8 O13
U 1 1 54F55289
P 4450 4050
F 0 "O13" H 4650 3650 60  0000 C CNN
F 1 "ITR8307/TR8" V 4250 4150 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 4450 4050 60  0001 C CNN
F 3 "" H 4450 4050 60  0000 C CNN
	1    4450 4050
	1    0    0    -1  
$EndComp
$Comp
L R R25
U 1 1 54F5528F
P 4550 3150
F 0 "R25" V 4450 3150 50  0000 C CNN
F 1 "10k" V 4557 3151 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 4480 3150 30  0001 C CNN
F 3 "" H 4550 3150 30  0000 C CNN
	1    4550 3150
	1    0    0    -1  
$EndComp
Text GLabel 4700 2900 3    60   Input ~ 0
VDD50
$Comp
L R R26
U 1 1 54F55297
P 4800 3900
F 0 "R26" V 4880 3900 50  0000 C CNN
F 1 "150" V 4807 3901 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 4730 3900 30  0001 C CNN
F 3 "" H 4800 3900 30  0000 C CNN
	1    4800 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 3300 4550 3450
Wire Wire Line
	4800 2900 4550 2900
Wire Wire Line
	4800 2900 4800 3750
Wire Wire Line
	4800 4050 4800 4350
Wire Wire Line
	4800 4350 4650 4350
Wire Wire Line
	4550 3400 4500 3400
$Comp
L ITR8307/TR8 O14
U 1 1 54F552A4
P 5100 4050
F 0 "O14" H 5300 3650 60  0000 C CNN
F 1 "ITR8307/TR8" V 4900 4150 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 5100 4050 60  0001 C CNN
F 3 "" H 5100 4050 60  0000 C CNN
	1    5100 4050
	1    0    0    -1  
$EndComp
$Comp
L R R27
U 1 1 54F552AA
P 5200 3150
F 0 "R27" V 5100 3150 50  0000 C CNN
F 1 "10k" V 5207 3151 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 5130 3150 30  0001 C CNN
F 3 "" H 5200 3150 30  0000 C CNN
	1    5200 3150
	1    0    0    -1  
$EndComp
Text GLabel 5350 2900 3    60   Input ~ 0
VDD50
$Comp
L R R28
U 1 1 54F552B2
P 5450 3900
F 0 "R28" V 5530 3900 50  0000 C CNN
F 1 "150" V 5457 3901 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 5380 3900 30  0001 C CNN
F 3 "" H 5450 3900 30  0000 C CNN
	1    5450 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 3300 5200 3450
Wire Wire Line
	5450 2900 5200 2900
Wire Wire Line
	5450 2900 5450 3750
Wire Wire Line
	5450 4050 5450 4350
Wire Wire Line
	5450 4350 5300 4350
Wire Wire Line
	5200 3400 5150 3400
Connection ~ 1850 2450
Wire Wire Line
	6650 6700 7100 6700
Wire Wire Line
	6950 6700 6950 6350
Text GLabel 6650 7400 2    60   Input ~ 0
GND
Wire Wire Line
	6350 6900 6300 6900
Text GLabel 6300 6900 0    60   Input ~ 0
SENS_NFET
$Comp
L R R30
U 1 1 54F6ECC7
P 6350 7150
F 0 "R30" V 6430 7150 50  0000 C CNN
F 1 "10k" V 6357 7151 50  0000 C CNN
F 2 "w_smd_resistors:r_0603" V 6280 7150 30  0001 C CNN
F 3 "" H 6350 7150 30  0000 C CNN
	1    6350 7150
	1    0    0    -1  
$EndComp
Text Label 6950 6350 2    60   ~ 0
MOS_DRAIN
Wire Wire Line
	6650 7000 6650 7400
Wire Wire Line
	6650 7400 6350 7400
Text Label 1750 2550 3    60   ~ 0
MOS_DRAIN
Wire Wire Line
	4150 1000 4150 1850
Wire Wire Line
	4150 1000 3900 1000
Wire Wire Line
	3500 1000 3250 1000
Wire Wire Line
	3500 1000 3500 1850
Wire Wire Line
	2200 1000 1950 1000
Wire Wire Line
	2850 1000 2600 1000
Wire Wire Line
	2850 1000 2850 1850
Wire Wire Line
	2200 1000 2200 1850
Text Label 2400 2550 3    60   ~ 0
MOS_DRAIN
Text Label 3050 2550 3    60   ~ 0
MOS_DRAIN
Text Label 3700 2550 3    60   ~ 0
MOS_DRAIN
Text Label 4350 2550 3    60   ~ 0
MOS_DRAIN
Text Label 5000 2550 3    60   ~ 0
MOS_DRAIN
Text Label 5650 2550 3    60   ~ 0
MOS_DRAIN
Text Label 6300 2550 3    60   ~ 0
MOS_DRAIN
Text Label 1750 4450 3    60   ~ 0
MOS_DRAIN
Text Label 2400 4450 3    60   ~ 0
MOS_DRAIN
Text Label 3050 4450 3    60   ~ 0
MOS_DRAIN
Text Label 3700 4450 3    60   ~ 0
MOS_DRAIN
Text Label 4350 4450 3    60   ~ 0
MOS_DRAIN
Text Label 5000 4450 3    60   ~ 0
MOS_DRAIN
Wire Wire Line
	1750 2550 1750 2450
Wire Wire Line
	1750 2450 1900 2450
Wire Wire Line
	2400 2550 2400 2450
Wire Wire Line
	2400 2450 2550 2450
Wire Wire Line
	3050 2450 3200 2450
Wire Wire Line
	3050 2450 3050 2550
Wire Wire Line
	3700 2450 3850 2450
Wire Wire Line
	3700 2450 3700 2550
Wire Wire Line
	4350 2450 4500 2450
Wire Wire Line
	4350 2550 4350 2450
Wire Wire Line
	5000 2450 5000 2550
Wire Wire Line
	5000 2450 5150 2450
Wire Wire Line
	5650 2450 5650 2550
Wire Wire Line
	5650 2450 5800 2450
$Comp
L ITR8307/TR8 O8
U 1 1 54F53BEA
P 6400 2150
F 0 "O8" H 6600 1750 60  0000 C CNN
F 1 "ITR8307/TR8" V 6200 2250 60  0000 C CNN
F 2 "freaquency:w_freaquency_smd_opto_ITR8307" H 6400 2150 60  0001 C CNN
F 3 "" H 6400 2150 60  0000 C CNN
	1    6400 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 2450 6300 2550
Wire Wire Line
	6300 2450 6450 2450
Wire Wire Line
	1750 4350 1750 4450
Wire Wire Line
	1750 4350 1900 4350
Wire Wire Line
	2400 4350 2400 4450
Wire Wire Line
	2400 4350 2550 4350
Wire Wire Line
	3050 4350 3050 4450
Wire Wire Line
	3050 4350 3200 4350
Wire Wire Line
	3700 4350 3700 4450
Wire Wire Line
	3700 4350 3850 4350
Wire Wire Line
	4350 4350 4350 4450
Wire Wire Line
	4350 4350 4500 4350
Wire Wire Line
	5000 4350 5000 4450
Wire Wire Line
	5000 4350 5150 4350
Text GLabel 1650 6550 2    60   Input ~ 0
ADC0
Text GLabel 1650 6450 2    60   Input ~ 0
ADC1
Text GLabel 1650 6050 2    60   Input ~ 0
ADC2
Text GLabel 1650 6150 2    60   Input ~ 0
ADC3
Text GLabel 1650 6250 2    60   Input ~ 0
ADC4
Text GLabel 1650 6350 2    60   Input ~ 0
ADC5
Text GLabel 1650 7050 2    60   Input ~ 0
ADC6
Text GLabel 1650 7350 2    60   Input ~ 0
ADC9
Text GLabel 1650 7150 2    60   Input ~ 0
ADC7
Text GLabel 1650 7250 2    60   Input ~ 0
ADC8
Text GLabel 1650 6950 2    60   Input ~ 0
ADC10
Text GLabel 1650 6850 2    60   Input ~ 0
ADC11
Text GLabel 1650 6750 2    60   Input ~ 0
ADC12
Text GLabel 1650 6650 2    60   Input ~ 0
ADC13
Text GLabel 1650 7950 2    60   Input ~ 0
VDD50
Text GLabel 1650 7650 2    60   Input ~ 0
SENS_NFET
Wire Wire Line
	1650 7450 1550 7450
Wire Wire Line
	1650 7550 1550 7550
Wire Wire Line
	1650 7650 1550 7650
Wire Wire Line
	1650 7750 1550 7750
Wire Wire Line
	1550 6450 1650 6450
Wire Wire Line
	1650 6550 1550 6550
Wire Wire Line
	1550 6650 1650 6650
Wire Wire Line
	1650 6750 1550 6750
Wire Wire Line
	1550 6850 1650 6850
Wire Wire Line
	1650 6950 1550 6950
Wire Wire Line
	1550 7050 1650 7050
Wire Wire Line
	1550 7150 1650 7150
Wire Wire Line
	1650 7250 1550 7250
Wire Wire Line
	1550 7350 1650 7350
$Comp
L HEADER_2 J5
U 1 1 54FDE8EA
P 7200 6900
F 0 "J5" H 7200 6700 60  0000 C CNN
F 1 "HEADER_2" V 7400 6900 60  0000 C CNN
F 2 "w_pin_strip:pin_strip_2" H 7200 6900 60  0001 C CNN
F 3 "" H 7200 6900 60  0000 C CNN
	1    7200 6900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 6700 7100 6850
Connection ~ 6950 6700
Wire Wire Line
	7100 6950 7100 7100
Wire Wire Line
	7100 7100 6650 7100
Wire Wire Line
	6350 6850 6350 7000
Wire Wire Line
	6350 6850 6450 6850
Connection ~ 6650 7100
$Comp
L HEADER_20 J1
U 1 1 550250F6
P 1450 7000
F 0 "J1" H 1850 7100 60  0000 C CNN
F 1 "HEADER_20" H 1900 6900 60  0000 C CNN
F 2 "" H 1450 7000 60  0000 C CNN
F 3 "" H 1450 7000 60  0000 C CNN
	1    1450 7000
	-1   0    0    1   
$EndComp
Text GLabel 1650 7850 2    60   Input ~ 0
GND
Wire Wire Line
	1550 7850 1650 7850
Wire Wire Line
	1650 7950 1550 7950
Text GLabel 1650 7450 2    60   Input ~ 0
SPI2_CLK
Text GLabel 1650 7550 2    60   Input ~ 0
LED_LE
Text GLabel 1650 7750 2    60   Input ~ 0
SPI2_MOSI
Wire Wire Line
	1650 6350 1550 6350
Wire Wire Line
	1550 6250 1650 6250
Wire Wire Line
	1650 6150 1550 6150
Wire Wire Line
	1550 6050 1650 6050
Text GLabel 3050 6750 0    60   Input ~ 0
SPI2_MOSI
Text GLabel 3050 6950 0    60   Input ~ 0
SPI2_CLK
Text GLabel 3050 7150 0    60   Input ~ 0
LED_LE
$Comp
L LED D1
U 1 1 5502F929
P 4400 6650
F 0 "D1" H 4400 6750 50  0000 C CNN
F 1 "LED" H 4400 6550 50  0000 C CNN
F 2 "" H 4400 6650 60  0000 C CNN
F 3 "" H 4400 6650 60  0000 C CNN
	1    4400 6650
	-1   0    0    1   
$EndComp
$Comp
L LED D2
U 1 1 5502FADE
P 4700 6750
F 0 "D2" H 4700 6850 50  0000 C CNN
F 1 "LED" H 4700 6650 50  0000 C CNN
F 2 "" H 4700 6750 60  0000 C CNN
F 3 "" H 4700 6750 60  0000 C CNN
	1    4700 6750
	-1   0    0    1   
$EndComp
$Comp
L LED D3
U 1 1 5502FB98
P 5000 6850
F 0 "D3" H 5000 6950 50  0000 C CNN
F 1 "LED" H 5000 6750 50  0000 C CNN
F 2 "" H 5000 6850 60  0000 C CNN
F 3 "" H 5000 6850 60  0000 C CNN
	1    5000 6850
	-1   0    0    1   
$EndComp
$Comp
L LED D4
U 1 1 5502FC7B
P 4400 6950
F 0 "D4" H 4400 7050 50  0000 C CNN
F 1 "LED" H 4400 6850 50  0000 C CNN
F 2 "" H 4400 6950 60  0000 C CNN
F 3 "" H 4400 6950 60  0000 C CNN
	1    4400 6950
	-1   0    0    1   
$EndComp
$Comp
L LED D5
U 1 1 5502FC81
P 4700 7050
F 0 "D5" H 4700 7150 50  0000 C CNN
F 1 "LED" H 4700 6950 50  0000 C CNN
F 2 "" H 4700 7050 60  0000 C CNN
F 3 "" H 4700 7050 60  0000 C CNN
	1    4700 7050
	-1   0    0    1   
$EndComp
$Comp
L LED D6
U 1 1 5502FC87
P 5000 7150
F 0 "D6" H 5000 7250 50  0000 C CNN
F 1 "LED" H 5000 7050 50  0000 C CNN
F 2 "" H 5000 7150 60  0000 C CNN
F 3 "" H 5000 7150 60  0000 C CNN
	1    5000 7150
	-1   0    0    1   
$EndComp
$Comp
L LED D7
U 1 1 5502FE82
P 4400 7250
F 0 "D7" H 4400 7350 50  0000 C CNN
F 1 "LED" H 4400 7150 50  0000 C CNN
F 2 "" H 4400 7250 60  0000 C CNN
F 3 "" H 4400 7250 60  0000 C CNN
	1    4400 7250
	-1   0    0    1   
$EndComp
$Comp
L LED D8
U 1 1 5502FE88
P 4700 7350
F 0 "D8" H 4700 7450 50  0000 C CNN
F 1 "LED" H 4700 7250 50  0000 C CNN
F 2 "" H 4700 7350 60  0000 C CNN
F 3 "" H 4700 7350 60  0000 C CNN
	1    4700 7350
	-1   0    0    1   
$EndComp
$Comp
L LED D9
U 1 1 5502FE8E
P 5000 7450
F 0 "D9" H 5000 7550 50  0000 C CNN
F 1 "LED" H 5000 7350 50  0000 C CNN
F 2 "" H 5000 7450 60  0000 C CNN
F 3 "" H 5000 7450 60  0000 C CNN
	1    5000 7450
	-1   0    0    1   
$EndComp
$Comp
L LED D10
U 1 1 5502FE94
P 4400 7550
F 0 "D10" H 4400 7650 50  0000 C CNN
F 1 "LED" H 4400 7450 50  0000 C CNN
F 2 "" H 4400 7550 60  0000 C CNN
F 3 "" H 4400 7550 60  0000 C CNN
	1    4400 7550
	-1   0    0    1   
$EndComp
$Comp
L LED D11
U 1 1 5502FE9A
P 4700 7650
F 0 "D11" H 4700 7750 50  0000 C CNN
F 1 "LED" H 4700 7550 50  0000 C CNN
F 2 "" H 4700 7650 60  0000 C CNN
F 3 "" H 4700 7650 60  0000 C CNN
	1    4700 7650
	-1   0    0    1   
$EndComp
$Comp
L LED D12
U 1 1 5502FEA0
P 5000 7750
F 0 "D12" H 5000 7850 50  0000 C CNN
F 1 "LED" H 5000 7650 50  0000 C CNN
F 2 "" H 5000 7750 60  0000 C CNN
F 3 "" H 5000 7750 60  0000 C CNN
	1    5000 7750
	-1   0    0    1   
$EndComp
$Comp
L LED D13
U 1 1 5503060A
P 4400 7850
F 0 "D13" H 4400 7950 50  0000 C CNN
F 1 "LED" H 4400 7750 50  0000 C CNN
F 2 "" H 4400 7850 60  0000 C CNN
F 3 "" H 4400 7850 60  0000 C CNN
	1    4400 7850
	-1   0    0    1   
$EndComp
$Comp
L LED D14
U 1 1 55030610
P 4700 7950
F 0 "D14" H 4700 8050 50  0000 C CNN
F 1 "LED" H 4700 7850 50  0000 C CNN
F 2 "" H 4700 7950 60  0000 C CNN
F 3 "" H 4700 7950 60  0000 C CNN
	1    4700 7950
	-1   0    0    1   
$EndComp
$Comp
L LED D15
U 1 1 55030616
P 5000 8050
F 0 "D15" H 5000 8150 50  0000 C CNN
F 1 "LED" H 5000 7950 50  0000 C CNN
F 2 "" H 5000 8050 60  0000 C CNN
F 3 "" H 5000 8050 60  0000 C CNN
	1    5000 8050
	-1   0    0    1   
$EndComp
$Comp
L LED D16
U 1 1 5503061C
P 4400 8150
F 0 "D16" H 4400 8250 50  0000 C CNN
F 1 "LED" H 4400 8050 50  0000 C CNN
F 2 "" H 4400 8150 60  0000 C CNN
F 3 "" H 4400 8150 60  0000 C CNN
	1    4400 8150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 6650 4200 6650
Wire Wire Line
	4150 6750 4500 6750
Wire Wire Line
	4150 6850 4800 6850
Wire Wire Line
	4150 6950 4200 6950
Wire Wire Line
	4150 7050 4500 7050
Wire Wire Line
	4150 7150 4800 7150
Wire Wire Line
	4150 7250 4200 7250
Wire Wire Line
	4150 7350 4500 7350
Wire Wire Line
	4150 7450 4800 7450
Wire Wire Line
	4150 7550 4200 7550
Wire Wire Line
	4150 7650 4500 7650
Wire Wire Line
	4150 7750 4800 7750
Wire Wire Line
	4150 7850 4200 7850
Wire Wire Line
	4500 7950 4150 7950
Wire Wire Line
	4800 8050 4150 8050
Wire Wire Line
	4150 8150 4200 8150
Wire Wire Line
	5200 8150 4600 8150
Wire Wire Line
	5200 6200 5200 8150
Connection ~ 5200 8050
Connection ~ 5200 7750
Connection ~ 5200 7450
Connection ~ 5200 7150
Wire Wire Line
	4600 6650 5200 6650
Connection ~ 5200 6850
Wire Wire Line
	4900 6750 5200 6750
Connection ~ 5200 6750
Wire Wire Line
	4600 6950 5200 6950
Connection ~ 5200 6950
Wire Wire Line
	4900 7050 5200 7050
Wire Wire Line
	5200 7050 5200 7000
Wire Wire Line
	4600 7250 5200 7250
Connection ~ 5200 7250
Connection ~ 5200 7350
Wire Wire Line
	4600 7550 5200 7550
Connection ~ 5200 7550
Wire Wire Line
	4900 7650 5200 7650
Connection ~ 5200 7650
Wire Wire Line
	4600 7850 5200 7850
Connection ~ 5200 7850
Wire Wire Line
	4900 7950 5200 7950
Connection ~ 5200 7950
Text GLabel 5200 6650 2    60   Input ~ 0
VDD50
Text GLabel 3600 6200 1    60   Input ~ 0
GND
Wire Wire Line
	3700 6200 3600 6200
Wire Wire Line
	3900 6200 5200 6200
Connection ~ 5200 6650
$Comp
L C C1
U 1 1 55035D35
P 3900 6050
F 0 "C1" V 3800 5900 50  0000 L CNN
F 1 "10u" V 3800 6150 50  0000 L CNN
F 2 "" H 3938 5900 30  0000 C CNN
F 3 "" H 3900 6050 60  0000 C CNN
	1    3900 6050
	0    1    1    0   
$EndComp
Wire Wire Line
	3700 5300 3700 6200
Wire Wire Line
	4100 6050 4100 6200
Connection ~ 4100 6200
$Comp
L POT RV1
U 1 1 55036C48
P 3400 5450
F 0 "RV1" H 3400 5350 50  0000 C CNN
F 1 "POT" H 3400 5450 50  0000 C CNN
F 2 "" H 3400 5450 60  0000 C CNN
F 3 "" H 3400 5450 60  0000 C CNN
	1    3400 5450
	1    0    0    1   
$EndComp
$Comp
L R R29
U 1 1 55036DA3
P 3400 5900
F 0 "R29" H 3250 5900 50  0000 C CNN
F 1 "1k" V 3400 5900 50  0000 C CNN
F 2 "" V 3330 5900 30  0000 C CNN
F 3 "" H 3400 5900 30  0000 C CNN
	1    3400 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 5600 3400 5750
Wire Wire Line
	3650 5450 3700 5450
Connection ~ 3700 6050
Wire Wire Line
	3150 5450 3150 5300
Wire Wire Line
	3150 5300 3700 5300
Connection ~ 3700 5450
Wire Wire Line
	3400 6050 3400 6200
Text GLabel 3050 7350 0    60   Input ~ 0
GND
Wire Wire Line
	5200 7350 4900 7350
Connection ~ 5200 7050
$Comp
L STP16CP05 U1
U 1 1 55062A9A
P 3700 7500
F 0 "U1" H 3950 6700 60  0000 C CNN
F 1 "STP16CP05" V 3550 7200 60  0000 C CNN
F 2 "" H 4100 7500 60  0000 C CNN
F 3 "" H 4100 7500 60  0000 C CNN
	1    3700 7500
	1    0    0    -1  
$EndComp
Connection ~ 6350 6900
Wire Wire Line
	6350 7400 6350 7300
Wire Wire Line
	3700 6050 3750 6050
Wire Wire Line
	4100 6050 4050 6050
Wire Wire Line
	1950 1000 1950 1100
Connection ~ 1950 1500
Connection ~ 2600 1500
Wire Wire Line
	2600 1000 2600 1100
Wire Wire Line
	3250 1000 3250 1100
Connection ~ 3250 1500
Connection ~ 3900 1500
Wire Wire Line
	3900 1000 3900 1100
Wire Wire Line
	4550 1000 4550 1100
Connection ~ 4550 1500
Wire Wire Line
	5200 1000 5200 1100
Connection ~ 5200 1500
Wire Wire Line
	5850 1000 5850 1100
Connection ~ 5850 1500
Connection ~ 6500 1500
Wire Wire Line
	6500 1000 6500 1100
Wire Wire Line
	5200 2900 5200 3000
Connection ~ 5200 3400
Connection ~ 4550 3400
Wire Wire Line
	4550 2900 4550 3000
Wire Wire Line
	3900 2900 3900 3000
Connection ~ 3900 3400
Connection ~ 3250 3400
Wire Wire Line
	3250 2900 3250 3000
Wire Wire Line
	2600 2900 2600 3000
Connection ~ 2600 3400
Connection ~ 1950 3400
Wire Wire Line
	1950 2900 1950 3000
$Comp
L drill U2
U 1 1 55145959
P 6600 8350
F 0 "U2" H 6600 8450 60  0000 C CNN
F 1 "drill" H 6600 8250 60  0000 C CNN
F 2 "" H 6600 8350 60  0000 C CNN
F 3 "" H 6600 8350 60  0000 C CNN
	1    6600 8350
	1    0    0    -1  
$EndComp
$Comp
L drill U3
U 1 1 5514B5C3
P 6800 8350
F 0 "U3" H 6800 8450 60  0000 C CNN
F 1 "drill" H 6800 8250 60  0000 C CNN
F 2 "" H 6800 8350 60  0000 C CNN
F 3 "" H 6800 8350 60  0000 C CNN
	1    6800 8350
	1    0    0    -1  
$EndComp
$Comp
L drill U4
U 1 1 5514B7C8
P 7000 8350
F 0 "U4" H 7000 8450 60  0000 C CNN
F 1 "drill" H 7000 8250 60  0000 C CNN
F 2 "" H 7000 8350 60  0000 C CNN
F 3 "" H 7000 8350 60  0000 C CNN
	1    7000 8350
	1    0    0    -1  
$EndComp
$Comp
L FET_N_GDS Q1
U 1 1 55176027
P 6600 6850
F 0 "Q1" H 6503 7100 70  0000 C CNN
F 1 "FET_N_GDS" H 6550 7200 60  0000 C CNN
F 2 "" H 6600 6850 60  0000 C CNN
F 3 "" H 6600 6850 60  0000 C CNN
	1    6600 6850
	1    0    0    -1  
$EndComp
Connection ~ 2500 2450
Connection ~ 3150 2450
Connection ~ 3800 2450
Connection ~ 4450 2450
Connection ~ 5100 2450
Connection ~ 5750 2450
Connection ~ 6400 2450
Connection ~ 1850 4350
Connection ~ 2500 4350
Connection ~ 3150 4350
Connection ~ 3800 4350
Connection ~ 4450 4350
Connection ~ 5100 4350
$EndSCHEMATC
