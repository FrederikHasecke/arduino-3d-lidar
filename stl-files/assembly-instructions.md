# Assembly Instructions

## 3D Printing

![Recommended Printing Orientation](Images/3dprint_orientation.png)


## Arduino UNO Wiring Instructions

[!CAUTION] VL53L0X XSHUT is not 5V tolerant and can be damaged by 5V! 

The wiring instructions for the 8 vl53l0x sensors, the motor, the motor driver, the microcontroller, the slip ring, and the motor comparator speed sensor module are as follows:

![Wiring Instructions](Images/wiring_diagram.png)

Function | Arduino pin | Notes
---------|-------------|------------
VL53L0X Sensor 0 XSHUT	| not used in code | Sensor 1 stays on default I2C address 41
VL53L0X Sensor 1 XSHUT	| 4	| reassigned to I2C address 42
VL53L0X Sensor 2 XSHUT	| 5	| reassigned to 43
VL53L0X Sensor 3 XSHUT	| 6	| reassigned to 44
VL53L0X Sensor 4 XSHUT	| 7 |	reassigned to 45
VL53L0X Sensor 5 XSHUT	| 8 |	reassigned to 46
VL53L0X Sensor 6 XSHUT	| 9 |	reassigned to 47
VL53L0X Sensor 7 XSHUT	| 10 |	reassigned to 48
Motor driver enable enA	| 11 |	PWM speed control
Motor driver input in1	| 12 |	direction control
Motor driver input in2	| 13 |	direction control
Rotation encoder output	| 2	| interrupt input on rising edge
USB serial	| Serial at 115200 |	data output to host computer

## ESP32 Wiring Instructions

[!CAUTION] VL53L0X XSHUT is not 5V tolerant and can be damaged by 5V! 

![Wiring Instructions](Images/ESP32-WROOM-wiring-diagram.png)

Function |	ESP32 pin |	Notes
---------|-------------|------------
VL53L0X Sensor 0 XSHUT |	0 | 	reassigned to I2C address 42
VL53L0X Sensor 1 XSHUT |	4 | 	reassigned to 43
VL53L0X Sensor 2 XSHUT |	16 | 	reassigned to 44
VL53L0X Sensor 3 XSHUT |	17 | 	reassigned to 45
VL53L0X Sensor 4 XSHUT |	5 | 	reassigned to 46
VL53L0X Sensor 5 XSHUT |	18 | 	reassigned to 47
VL53L0X Sensor 6 XSHUT |	19 | 	reassigned to 48
VL53L0X Sensor 7 XSHUT |	23 | 	kept at default I2C address 41
Motor driver enable enA |	25 |	PWM speed control
Motor driver input in1 |	33 |	direction control
Motor driver input in2 |	32 |	direction control
Rotation encoder output |	13 |	interrupt input on rising edge
USB serial |	Serial at 115200 |	data output to host computer

