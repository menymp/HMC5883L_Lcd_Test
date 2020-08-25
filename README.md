# HMC5883L_Lcd_Test
Simple script test for HMC5883l three axis magnetometer in C, the axis registers are shown in the LCD display.
The targeted microcontroler is the Microchip 18F4550

The wiring for the LCD is the folowing:
LCD             18F4550
LCD_RS_PIN      PIN_E0
LCD_RW_PIN      PIN_E1  
LCD_ENABLE_PIN  PIN_E2  
LCD_DATA4       PIN_D4 
LCD_DATA5       PIN_D5 
LCD_DATA6       PIN_D6 
LCD_DATA7       PIN_D7 

The I2C wiring for the sensor is the folowing:
HMC5883L
SDA      ---------- RB0
SCL      ---------- RB1

