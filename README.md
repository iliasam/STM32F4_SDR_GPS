# STM32F4_SDR_GPS

Software GPS receiver for STM32F4  
See article (in Russian): https://habr.com/ru/articles/789382/  
You can find same project for **ESP32** here: https://github.com/iliasam/ESP32_SDR_GPS  
  
This is a demo project of STM32-based SDR GPS receiver.  
No additional factory/commercial "black box" GPS receiver is needed here - just connect MAX2769 to the STM32 MCU.  
All "raw" GPS signal processing is done at the STM32.  
Results of the GPS receiving are send to the UART:   
<img src="https://github.com/iliasam/STM32F4_SDR_GPS/blob/develop/Images/Results.png" width="600">  
That how receiver is looking:  
<img src="https://github.com/iliasam/STM32F4_SDR_GPS/blob/develop/Images/photo_stm32a.jpg" width="500">  
  
<img src="https://github.com/iliasam/STM32F4_SDR_GPS/blob/develop/Images/Structure_stm32.png" width="800">  
  
**Video about this project:**  
https://youtu.be/1hBnaDsQgMc  
  
Configured for STM32F4-DISCOVERY dev. board.  
-Os optimization is used here  

User set PRN codes of 4 satellites in code - before compiling FW.  
User can enter Doppler frequency offset to make acquisition much faster.  
  
RF frontend pinout (See config.h):  
#define SPI_CLK_PIN             GPIO_Pin_13 //MAX2769 - CLKOUT  
#define SPI_MOSI_PIN            GPIO_Pin_15 //MAX2769 - I1  
Both - GPIOB  

  
Part of the code is taken from these projects:  
https://github.com/tomojitakasu/RTKLIB  
https://github.com/taroz/GNSS-SDRLIB  
