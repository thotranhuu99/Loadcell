The project includes the .ioc file for CubeMX, after generate code for KEIL don't forget to replace the main.c with the corresponding file in git branch.

Place i2c.c in the same folder with main.c.
Compile and the generate hex file for using with STM32F103C8T6.
Note: The main.c only included nessesary functions for reading the loadcell's differential voltage output after being amplified, calculate the offset and display in LCD. 
These funtions can be called and used in the while loop or in DMA interrupt.
