EESchema Schematic File Version 4
LIBS:PomodoroTomatoTimer-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Pomodoro Tomato Timer"
Date "2019-07-17"
Rev "v1"
Comp "Engin Subaşı"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_ST_STM32F0:STM32F030F4Px U1
U 1 1 5D2E3CF4
P 3900 2100
F 0 "U1" H 4200 1250 50  0000 C CNN
F 1 "STM32F030F4Px" H 4450 1150 50  0000 C CNN
F 2 "Package_SO:TSSOP-20_4.4x6.5mm_P0.65mm" H 3500 1400 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 3900 2100 50  0001 C CNN
	1    3900 2100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0101
U 1 1 5D2E68AE
P 3900 1300
F 0 "#PWR0101" H 3900 1150 50  0001 C CNN
F 1 "VDD" H 3917 1473 50  0000 C CNN
F 2 "" H 3900 1300 50  0001 C CNN
F 3 "" H 3900 1300 50  0001 C CNN
	1    3900 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5D2E6CBF
P 3900 3000
F 0 "#PWR0102" H 3900 2750 50  0001 C CNN
F 1 "GND" H 3905 2827 50  0000 C CNN
F 2 "" H 3900 3000 50  0001 C CNN
F 3 "" H 3900 3000 50  0001 C CNN
	1    3900 3000
	1    0    0    -1  
$EndComp
NoConn ~ 3400 2400
NoConn ~ 3400 2500
$Comp
L Device:R_Small R2
U 1 1 5D2E7193
P 2950 1800
F 0 "R2" V 2850 1800 50  0000 C CNN
F 1 "10k 0603" V 2800 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2950 1800 50  0001 C CNN
F 3 "~" H 2950 1800 50  0001 C CNN
	1    2950 1800
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5D2E7693
P 2750 1800
F 0 "#PWR0103" H 2750 1550 50  0001 C CNN
F 1 "GND" V 2755 1672 50  0000 R CNN
F 2 "" H 2750 1800 50  0001 C CNN
F 3 "" H 2750 1800 50  0001 C CNN
	1    2750 1800
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R1
U 1 1 5D2E7C2F
P 2950 1600
F 0 "R1" V 2750 1600 50  0000 C CNN
F 1 "10k 0603" V 2800 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2950 1600 50  0001 C CNN
F 3 "~" H 2950 1600 50  0001 C CNN
	1    2950 1600
	0    1    1    0   
$EndComp
Wire Wire Line
	3050 1600 3400 1600
Wire Wire Line
	3050 1800 3400 1800
Wire Wire Line
	2750 1800 2850 1800
Wire Wire Line
	3900 1300 3900 1350
Wire Wire Line
	4000 1400 4000 1350
Wire Wire Line
	4000 1350 3900 1350
Connection ~ 3900 1350
Wire Wire Line
	3900 1350 3900 1400
$Comp
L power:VDD #PWR0104
U 1 1 5D2E901F
P 2750 1600
F 0 "#PWR0104" H 2750 1450 50  0001 C CNN
F 1 "VDD" V 2768 1727 50  0000 L CNN
F 2 "" H 2750 1600 50  0001 C CNN
F 3 "" H 2750 1600 50  0001 C CNN
	1    2750 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2750 1600 2850 1600
Wire Wire Line
	3900 3000 3900 2900
$Comp
L Device:R_Small R3
U 1 1 5D2EA0DC
P 7750 1300
F 0 "R3" V 7550 1300 50  0000 C CNN
F 1 "1k 0603" V 7600 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7750 1300 50  0001 C CNN
F 3 "~" H 7750 1300 50  0001 C CNN
	1    7750 1300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5D2EAA10
P 7750 1600
F 0 "R4" V 7550 1600 50  0000 C CNN
F 1 "1k 0603" V 7600 1600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7750 1600 50  0001 C CNN
F 3 "~" H 7750 1600 50  0001 C CNN
	1    7750 1600
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5D2EAD87
P 7750 1900
F 0 "R5" V 7550 1900 50  0000 C CNN
F 1 "1k 0603" V 7600 1900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7750 1900 50  0001 C CNN
F 3 "~" H 7750 1900 50  0001 C CNN
	1    7750 1900
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5D2EB10E
P 7750 2200
F 0 "R6" V 7550 2200 50  0000 C CNN
F 1 "1k 0603" V 7600 2200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7750 2200 50  0001 C CNN
F 3 "~" H 7750 2200 50  0001 C CNN
	1    7750 2200
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5D2EB384
P 7750 2500
F 0 "R7" V 7550 2500 50  0000 C CNN
F 1 "1k 0603" V 7600 2500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7750 2500 50  0001 C CNN
F 3 "~" H 7750 2500 50  0001 C CNN
	1    7750 2500
	0    1    1    0   
$EndComp
$Comp
L Device:LED_Small D1
U 1 1 5D2EB80E
P 8300 1300
F 0 "D1" H 8150 1150 50  0000 C CNN
F 1 "LED" H 8150 1250 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 8300 1300 50  0001 C CNN
F 3 "~" V 8300 1300 50  0001 C CNN
	1    8300 1300
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D2
U 1 1 5D2EC725
P 8300 1600
F 0 "D2" H 8150 1450 50  0000 C CNN
F 1 "LED" H 8150 1550 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 8300 1600 50  0001 C CNN
F 3 "~" V 8300 1600 50  0001 C CNN
	1    8300 1600
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D3
U 1 1 5D2ECAF1
P 8300 1900
F 0 "D3" H 8150 1750 50  0000 C CNN
F 1 "LED" H 8150 1850 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 8300 1900 50  0001 C CNN
F 3 "~" V 8300 1900 50  0001 C CNN
	1    8300 1900
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D4
U 1 1 5D2ECE04
P 8300 2200
F 0 "D4" H 8150 2050 50  0000 C CNN
F 1 "LED" H 8150 2150 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 8300 2200 50  0001 C CNN
F 3 "~" V 8300 2200 50  0001 C CNN
	1    8300 2200
	-1   0    0    1   
$EndComp
$Comp
L Device:LED_Small D5
U 1 1 5D2ED25D
P 8300 2500
F 0 "D5" H 8150 2350 50  0000 C CNN
F 1 "LED" H 8150 2450 50  0000 C CNN
F 2 "LED_SMD:LED_0805_2012Metric" V 8300 2500 50  0001 C CNN
F 3 "~" V 8300 2500 50  0001 C CNN
	1    8300 2500
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5D2ED808
P 8700 2700
F 0 "#PWR0105" H 8700 2450 50  0001 C CNN
F 1 "GND" H 8705 2527 50  0000 C CNN
F 2 "" H 8700 2700 50  0001 C CNN
F 3 "" H 8700 2700 50  0001 C CNN
	1    8700 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1300 8700 1300
Wire Wire Line
	8700 1300 8700 1600
Wire Wire Line
	8400 2500 8700 2500
Connection ~ 8700 2500
Wire Wire Line
	8700 2500 8700 2700
Wire Wire Line
	8400 2200 8700 2200
Connection ~ 8700 2200
Wire Wire Line
	8700 2200 8700 2500
Wire Wire Line
	8400 1900 8700 1900
Connection ~ 8700 1900
Wire Wire Line
	8700 1900 8700 2200
Wire Wire Line
	8400 1600 8700 1600
Connection ~ 8700 1600
Wire Wire Line
	8700 1600 8700 1900
Wire Wire Line
	7850 1300 8200 1300
Wire Wire Line
	8200 1600 7850 1600
Wire Wire Line
	7850 1900 8200 1900
Wire Wire Line
	7850 2200 8200 2200
Wire Wire Line
	8200 2500 7850 2500
Wire Wire Line
	7650 1300 6850 1300
Wire Wire Line
	7650 1600 6850 1600
Wire Wire Line
	7650 1900 6850 1900
Wire Wire Line
	6850 2200 7650 2200
Wire Wire Line
	7650 2500 6850 2500
Text Label 7050 1300 0    50   ~ 0
LED1
Text Label 7050 1600 0    50   ~ 0
LED2
Text Label 7050 1900 0    50   ~ 0
LED3
Text Label 7050 2200 0    50   ~ 0
LED4
Text Label 7050 2500 0    50   ~ 0
LED5
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5D2FBDFB
P 7900 4100
F 0 "J1" H 7818 4317 50  0000 C CNN
F 1 "Coin Cell" H 7818 4226 50  0000 C CNN
F 2 "Battery:BatteryHolder_Keystone_103_1x20mm" H 7900 4100 50  0001 C CNN
F 3 "~" H 7900 4100 50  0001 C CNN
	1    7900 4100
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5D2FD01E
P 8250 4200
F 0 "#PWR0106" H 8250 3950 50  0001 C CNN
F 1 "GND" V 8255 4072 50  0000 R CNN
F 2 "" H 8250 4200 50  0001 C CNN
F 3 "" H 8250 4200 50  0001 C CNN
	1    8250 4200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8100 4100 8250 4100
Wire Wire Line
	8100 4200 8250 4200
$Comp
L Device:C_Small C3
U 1 1 5D2FE46E
P 8700 4150
F 0 "C3" H 8792 4196 50  0000 L CNN
F 1 "100nF 0603" H 8792 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8700 4150 50  0001 C CNN
F 3 "~" H 8700 4150 50  0001 C CNN
	1    8700 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5D2FEBA0
P 9350 4150
F 0 "C4" H 9442 4196 50  0000 L CNN
F 1 "330nF 0603" H 9442 4105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9350 4150 50  0001 C CNN
F 3 "~" H 9350 4150 50  0001 C CNN
	1    9350 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5D302430
P 1300 2100
F 0 "C1" H 1392 2146 50  0000 L CNN
F 1 "100nF 0603" H 1392 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1300 2100 50  0001 C CNN
F 3 "~" H 1300 2100 50  0001 C CNN
	1    1300 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5D302436
P 1950 2100
F 0 "C2" H 2042 2146 50  0000 L CNN
F 1 "330nF 0603" H 2042 2055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1950 2100 50  0001 C CNN
F 3 "~" H 1950 2100 50  0001 C CNN
	1    1950 2100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0107
U 1 1 5D3031F1
P 1300 1900
F 0 "#PWR0107" H 1300 1750 50  0001 C CNN
F 1 "VDD" H 1317 2073 50  0000 C CNN
F 2 "" H 1300 1900 50  0001 C CNN
F 3 "" H 1300 1900 50  0001 C CNN
	1    1300 1900
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0108
U 1 1 5D30350D
P 1950 1900
F 0 "#PWR0108" H 1950 1750 50  0001 C CNN
F 1 "VDD" H 1967 2073 50  0000 C CNN
F 2 "" H 1950 1900 50  0001 C CNN
F 3 "" H 1950 1900 50  0001 C CNN
	1    1950 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5D30386C
P 8700 4350
F 0 "#PWR0109" H 8700 4100 50  0001 C CNN
F 1 "GND" H 8705 4177 50  0000 C CNN
F 2 "" H 8700 4350 50  0001 C CNN
F 3 "" H 8700 4350 50  0001 C CNN
	1    8700 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5D303D7F
P 9350 4350
F 0 "#PWR0110" H 9350 4100 50  0001 C CNN
F 1 "GND" H 9355 4177 50  0000 C CNN
F 2 "" H 9350 4350 50  0001 C CNN
F 3 "" H 9350 4350 50  0001 C CNN
	1    9350 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 5D30402E
P 1300 2300
F 0 "#PWR0111" H 1300 2050 50  0001 C CNN
F 1 "GND" H 1305 2127 50  0000 C CNN
F 2 "" H 1300 2300 50  0001 C CNN
F 3 "" H 1300 2300 50  0001 C CNN
	1    1300 2300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5D3042D8
P 1950 2300
F 0 "#PWR0112" H 1950 2050 50  0001 C CNN
F 1 "GND" H 1955 2127 50  0000 C CNN
F 2 "" H 1950 2300 50  0001 C CNN
F 3 "" H 1950 2300 50  0001 C CNN
	1    1950 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 3950 8700 4050
Wire Wire Line
	8700 4250 8700 4350
Wire Wire Line
	9350 4350 9350 4250
Wire Wire Line
	9350 4050 9350 3950
Wire Wire Line
	1300 1900 1300 2000
Wire Wire Line
	1950 1900 1950 2000
Wire Wire Line
	1950 2200 1950 2300
Wire Wire Line
	1300 2300 1300 2200
$Comp
L power:VDDA #PWR0113
U 1 1 5D309E1F
P 8250 4100
F 0 "#PWR0113" H 8250 3950 50  0001 C CNN
F 1 "VDDA" V 8267 4228 50  0000 L CNN
F 2 "" H 8250 4100 50  0001 C CNN
F 3 "" H 8250 4100 50  0001 C CNN
	1    8250 4100
	0    1    1    0   
$EndComp
$Comp
L power:VDDA #PWR0114
U 1 1 5D30A635
P 8700 3950
F 0 "#PWR0114" H 8700 3800 50  0001 C CNN
F 1 "VDDA" H 8717 4123 50  0000 C CNN
F 2 "" H 8700 3950 50  0001 C CNN
F 3 "" H 8700 3950 50  0001 C CNN
	1    8700 3950
	1    0    0    -1  
$EndComp
$Comp
L power:VDDA #PWR0115
U 1 1 5D30AADC
P 9350 3950
F 0 "#PWR0115" H 9350 3800 50  0001 C CNN
F 1 "VDDA" H 9367 4123 50  0000 C CNN
F 2 "" H 9350 3950 50  0001 C CNN
F 3 "" H 9350 3950 50  0001 C CNN
	1    9350 3950
	1    0    0    -1  
$EndComp
Wire Notes Line
	1000 1000 5500 1000
Wire Notes Line
	5500 1000 5500 3500
Wire Notes Line
	5500 3500 1000 3500
Wire Notes Line
	1000 3500 1000 1000
Wire Notes Line
	7650 3650 10150 3650
Wire Notes Line
	10150 3650 10150 4650
Wire Notes Line
	10150 4650 7650 4650
Wire Notes Line
	7650 4650 7650 3650
Wire Notes Line
	6500 1000 9000 1000
Wire Notes Line
	9000 1000 9000 3000
Wire Notes Line
	9000 3000 6500 3000
Wire Notes Line
	6500 3000 6500 1000
Text Notes 1000 1000 0    50   ~ 0
MCU
Text Notes 7650 3650 0    50   ~ 0
POWER/BATTERY INPUT
Text Notes 6500 1000 0    50   ~ 0
LEDs
$Comp
L Switch:SW_DPST_x2 SW1
U 1 1 5D2FA5F3
P 1950 4250
F 0 "SW1" H 1950 4485 50  0000 C CNN
F 1 "SW_DPST_x2" H 1950 4394 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 1950 4250 50  0001 C CNN
F 3 "~" H 1950 4250 50  0001 C CNN
	1    1950 4250
	1    0    0    -1  
$EndComp
$Comp
L power:VDDA #PWR0116
U 1 1 5D2FCDD1
P 1650 4250
F 0 "#PWR0116" H 1650 4100 50  0001 C CNN
F 1 "VDDA" H 1667 4423 50  0000 C CNN
F 2 "" H 1650 4250 50  0001 C CNN
F 3 "" H 1650 4250 50  0001 C CNN
	1    1650 4250
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:BC807 Q3
U 1 1 5D3011CC
P 2950 4850
F 0 "Q3" H 3141 4804 50  0000 L CNN
F 1 "BC807" H 3141 4895 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3150 4775 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC807.pdf" H 2950 4850 50  0001 L CNN
	1    2950 4850
	-1   0    0    1   
$EndComp
$Comp
L Transistor_BJT:BC807 Q5
U 1 1 5D302918
P 3750 4850
F 0 "Q5" H 3941 4804 50  0000 L CNN
F 1 "BC807" H 3941 4895 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3950 4775 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC807.pdf" H 3750 4850 50  0001 L CNN
	1    3750 4850
	1    0    0    1   
$EndComp
Wire Wire Line
	2850 4650 2850 4250
Wire Wire Line
	2850 4250 3150 4250
Wire Wire Line
	3550 4250 3850 4250
Wire Wire Line
	3850 4250 3850 4650
Wire Wire Line
	3150 4850 3250 4850
$Comp
L Device:R_Small R10
U 1 1 5D306168
P 2850 5250
F 0 "R10" V 2750 5250 50  0000 C CNN
F 1 "47k 0603" V 2700 5250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2850 5250 50  0001 C CNN
F 3 "~" H 2850 5250 50  0001 C CNN
	1    2850 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R12
U 1 1 5D30755C
P 3850 5250
F 0 "R12" V 3750 5250 50  0000 C CNN
F 1 "47k 0603" V 3700 5250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3850 5250 50  0001 C CNN
F 3 "~" H 3850 5250 50  0001 C CNN
	1    3850 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 5050 2850 5100
Wire Wire Line
	2850 5100 3250 5100
Wire Wire Line
	3250 5100 3250 4850
Connection ~ 2850 5100
Wire Wire Line
	2850 5100 2850 5150
Connection ~ 3250 4850
Wire Wire Line
	3250 4850 3550 4850
Wire Wire Line
	3850 5050 3850 5100
Wire Wire Line
	3350 4550 3350 5100
Wire Wire Line
	3350 5100 3850 5100
Connection ~ 3850 5100
Wire Wire Line
	3850 5100 3850 5150
$Comp
L power:GND #PWR0117
U 1 1 5D32E974
P 2850 5450
F 0 "#PWR0117" H 2850 5200 50  0001 C CNN
F 1 "GND" H 2855 5277 50  0000 C CNN
F 2 "" H 2850 5450 50  0001 C CNN
F 3 "" H 2850 5450 50  0001 C CNN
	1    2850 5450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 5D32F0AF
P 3850 5450
F 0 "#PWR0118" H 3850 5200 50  0001 C CNN
F 1 "GND" H 3855 5277 50  0000 C CNN
F 2 "" H 3850 5450 50  0001 C CNN
F 3 "" H 3850 5450 50  0001 C CNN
	1    3850 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 5450 3850 5350
Wire Wire Line
	2850 5450 2850 5350
Connection ~ 2850 4250
Wire Wire Line
	1650 4250 1700 4250
$Comp
L Device:R_Small R9
U 1 1 5D34655C
P 2350 4650
F 0 "R9" V 2250 4650 50  0000 C CNN
F 1 "10k 0603" V 2200 4650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2350 4650 50  0001 C CNN
F 3 "~" H 2350 4650 50  0001 C CNN
	1    2350 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5D346B5E
P 2350 4850
F 0 "#PWR0119" H 2350 4600 50  0001 C CNN
F 1 "GND" H 2355 4677 50  0000 C CNN
F 2 "" H 2350 4850 50  0001 C CNN
F 3 "" H 2350 4850 50  0001 C CNN
	1    2350 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4850 2350 4750
Wire Wire Line
	2150 4250 2350 4250
Wire Wire Line
	2350 4550 2350 4250
Connection ~ 2350 4250
Wire Wire Line
	2350 4250 2850 4250
Text Label 2500 4250 0    50   ~ 0
BUTTON
$Comp
L power:VDD #PWR0120
U 1 1 5D34B680
P 5250 4150
F 0 "#PWR0120" H 5250 4000 50  0001 C CNN
F 1 "VDD" H 5267 4323 50  0000 C CNN
F 2 "" H 5250 4150 50  0001 C CNN
F 3 "" H 5250 4150 50  0001 C CNN
	1    5250 4150
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSS83P Q1
U 1 1 5D34EA81
P 2450 6350
F 0 "Q1" V 2793 6350 50  0000 C CNN
F 1 "BSS83P" V 2702 6350 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2650 6275 50  0001 L CIN
F 3 "http://www.farnell.com/datasheets/1835997.pdf" H 2450 6350 50  0001 L CNN
	1    2450 6350
	0    1    -1   0   
$EndComp
Wire Wire Line
	2250 6250 1700 6250
Wire Wire Line
	1700 6250 1700 4250
Connection ~ 1700 4250
Wire Wire Line
	1700 4250 1750 4250
Wire Notes Line
	4250 5750 4250 3750
Wire Notes Line
	2500 3750 2500 5750
Wire Notes Line
	2500 5750 4250 5750
Wire Notes Line
	2500 3750 4250 3750
Text Notes 2500 3750 0    50   ~ 0
IDEAL DIODE
Wire Wire Line
	5250 4250 5250 4150
Connection ~ 3850 4250
Wire Wire Line
	3850 4250 5250 4250
Wire Wire Line
	2650 6250 5250 6250
Wire Wire Line
	5250 6250 5250 4250
Connection ~ 5250 4250
$Comp
L Transistor_FET:2N7002 Q2
U 1 1 5D389512
P 2550 6850
F 0 "Q2" H 2756 6896 50  0000 L CNN
F 1 "2N7002" H 2756 6805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2750 6775 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7002.pdf" H 2550 6850 50  0001 L CNN
	1    2550 6850
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R8
U 1 1 5D38C6C1
P 1700 6450
F 0 "R8" V 1600 6450 50  0000 C CNN
F 1 "10k 0603" V 1550 6450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1700 6450 50  0001 C CNN
F 3 "~" H 1700 6450 50  0001 C CNN
	1    1700 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 6550 2450 6600
Wire Wire Line
	1700 6550 1700 6600
Wire Wire Line
	1700 6600 2450 6600
Connection ~ 2450 6600
Wire Wire Line
	2450 6600 2450 6650
Wire Wire Line
	1700 6350 1700 6250
Connection ~ 1700 6250
$Comp
L power:GND #PWR0121
U 1 1 5D39252F
P 2450 7250
F 0 "#PWR0121" H 2450 7000 50  0001 C CNN
F 1 "GND" H 2455 7077 50  0000 C CNN
F 2 "" H 2450 7250 50  0001 C CNN
F 3 "" H 2450 7250 50  0001 C CNN
	1    2450 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 7050 2450 7250
$Comp
L Device:R_Small R11
U 1 1 5D395304
P 2850 7050
F 0 "R11" V 2750 7050 50  0000 C CNN
F 1 "10k 0603" V 2700 7050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2850 7050 50  0001 C CNN
F 3 "~" H 2850 7050 50  0001 C CNN
	1    2850 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 6850 2850 6850
Wire Wire Line
	2850 6850 2850 6950
$Comp
L power:GND #PWR0122
U 1 1 5D3978D1
P 2850 7250
F 0 "#PWR0122" H 2850 7000 50  0001 C CNN
F 1 "GND" H 2855 7077 50  0000 C CNN
F 2 "" H 2850 7250 50  0001 C CNN
F 3 "" H 2850 7250 50  0001 C CNN
	1    2850 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 7250 2850 7150
Wire Wire Line
	2850 6850 3500 6850
Connection ~ 2850 6850
Text Label 3000 6850 0    50   ~ 0
POW_CTRL
$Comp
L Transistor_FET:BSS83P Q4
U 1 1 5D3BC797
P 3350 4350
F 0 "Q4" V 3693 4350 50  0000 C CNN
F 1 "BSS83P" V 3602 4350 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3550 4275 50  0001 L CIN
F 3 "http://www.farnell.com/datasheets/1835997.pdf" H 3350 4350 50  0001 L CNN
	1    3350 4350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4400 2100 4850 2100
Wire Wire Line
	4400 2000 4850 2000
Wire Wire Line
	4400 1900 4850 1900
Wire Wire Line
	4400 1800 4850 1800
Wire Wire Line
	4400 1700 4850 1700
Wire Wire Line
	4400 1600 4850 1600
Wire Wire Line
	4400 2600 4850 2600
Wire Wire Line
	4400 2500 4850 2500
Wire Wire Line
	4400 2400 4850 2400
Wire Wire Line
	4400 2300 4850 2300
Wire Wire Line
	4400 2200 4850 2200
Text Label 4550 2100 0    50   ~ 0
LED5
Text Label 4550 2200 0    50   ~ 0
LED4
Text Label 4550 2300 0    50   ~ 0
LED3
Wire Wire Line
	4400 2700 4850 2700
Wire Wire Line
	2950 2700 3400 2700
Text Label 3100 2700 0    50   ~ 0
LED2
Text Label 4550 2400 0    50   ~ 0
LED1
Text Label 4550 1600 0    50   ~ 0
POW_CTRL
Text Label 4550 2500 0    50   ~ 0
BUTTON
$Comp
L Graphic:Logo_Open_Hardware_Small #LOGO1
U 1 1 5D379F38
P 7050 5500
F 0 "#LOGO1" H 7050 5775 50  0001 C CNN
F 1 "Logo_Open_Hardware_Small" H 7050 5275 50  0001 C CNN
F 2 "" H 7050 5500 50  0001 C CNN
F 3 "~" H 7050 5500 50  0001 C CNN
	1    7050 5500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
