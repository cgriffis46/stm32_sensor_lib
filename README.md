# stm32_sensor_lib
 stm32 device static  libraries

Using the Static Libraries:

Right-click on the project name. Click "properties"
![image](https://github.com/cgriffis46/stm32_sensor_lib/assets/78368880/ff64dc42-cfe8-427c-8b1d-a041d2bdf2dd)

Under C/C++ Build->Settings->MCU G++ Linker, add the necessary Library files for your mcu,
![image](https://github.com/cgriffis46/stm32_sensor_lib/assets/78368880/007fc409-5ffc-40da-8267-08612be96abe)

Under C/C++ General->Paths and Symbols->Includes, add the path to the inc folder for your mcu, 
![image](https://github.com/cgriffis46/stm32_sensor_lib/assets/78368880/f1dba039-31b9-40e9-b96f-55420158c113)

Under C/C++ General->Paths and Symbols->Includes, add the path to the inc folder for your mcu,
![image](https://github.com/cgriffis46/stm32_sensor_lib/assets/78368880/4aa44f59-a1cf-4489-b460-c88111d96059)

Click "Apply and Close"

The STM32 Sensor libraries should be available for your project.

Change Log:
	2024-4-18 - Added xPM25 library as a FreeRTOS version of the PM25 driver. 