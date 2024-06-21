# [444 Final Project: Human Motion Detection (HMR)](https://github.com/ChenyiAXu/ECSE444-Final-Project/blob/main/444final/Core/Src/main.c)

A **CNN-based real-time Human Motion Detection (HMR)** system using **STM32 B-L475E-IOT01Ax kits**. The project is written in **C** and utilizes **USB, STM32AI, UART, and I2C** functionalities with **low power operation** optimization. The L4 board is worn on the wrist to imitate a watch, capable of classifying motions such as stationary, walking, running, jumping, clapping, and swimming.

## Features
- **Datasets**:
  - Stationary, walking, and running datasets are sourced from the [STM32 Cube AI GitHub Repository](https://github.com/STMicroelectronics/stm32ai-wiki).
  - Jumping, clapping, and swimming datasets are collected at a rate of **26Hz** using the **I2C feature** and saved as **CSV files** using the **USB feature**.
- **Low Power Optimization**: Confirmed using a **multimeter**.

## Training the Model
Adapted from ST, using **Keras** to process the motion sensing classifier model. For detailed steps, refer to the [Human Activity Recognition Notebook](https://github.com/STMicroelectronics/stm32ai-wiki/blob/master/AI_resources/HAR/Human_Activity_Recognition.ipynb). More layers and more functions are added in our prject. The model is saved into HDF5 file 'model.h5'

## Integration of AI Component to the project 
Using the source [STM32Cube.AI Application](https://wiki.st.com/stm32mcu/wiki/AI:How_to_perform_motion_sensing_on_STM32L4_IoTnode#Add_STM32Cube-AI_to_your_project). 
- Under artificial intelligence filter enable core
- Config X-cube_AI component to use exisitng Keras model
  - ''ADD Network''--> type to 'Keras' --> browse to select model
  - generate code
- Include CUBE AI header
- Declare neural network buffers
- Add AI bootstrapping functions
- create an 'argmax' function
- Call the AI_init function
  
## Pin Configuration
### I2C Configuration
- Software Pack -> Select Component -> Board Part -> AccGyr->LSM6DSL -> I2C
- Pinout & Configuration -> I2C: 'Fast mode'. speed Frequency '400 kHz'
- PB 10 `I2C2_SCL`
- PB 11 I2C2_SDA'
- PD 11 'GPIO_EXIT11'
- Software Packs -> X-Cube_MEMS1 -> Board Part Accgyr -> Platform Seeting ->I2C
### UART
- Connectivity -> USART1 -> Mode 'Asynchronous' -> Baud Rate to 115200 bit/s
- Ensure PB6 and PB7 are configured
### USB Feature 
- PA 11 'USB_OTG_FS_DM'
- PA 12 'USB_OTG_FS_DP'
- Connectivity ->USB_OTG_FS -> Mode 'Host Only' -> Avtivate_VBUS 'Disable'
- Middleware and Software Packs -> FATFS -> USB Disk
- Middleware and Software Packs -> USB_HOST -> class 'Mass Storage Host Class'-> Platform Setting -> Drive_VBUS_FS 'GPIO:Output', 'PA9'

## Low Power
- Remover the jumper at pin JP5
- Use multimeter to measure the current at the pin
