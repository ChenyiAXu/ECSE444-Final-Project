# [444 Final Project: Human Motion Detection (HMR)](https://github.com/ChenyiAXu/ECSE444-Final-Project/blob/main/444final/Core/Src/main.c)

A **CNN-based real-time Human Motion Detection (HMR)** system using **STM32 B-L475E-IOT01Ax kits**. This project, written in **C**, utilizes **USB, STM32AI, UART, and I2C** functionalities with a focus on **low power operation**. The L4 board, worn on the wrist like a watch, can classify various motions such as stationary, walking, running, jumping, clapping, and swimming.

## Features
- **Datasets**:
  - **Standard Motions**: Stationary, walking, and running datasets sourced from the [STM32 Cube AI GitHub Repository](https://github.com/STMicroelectronics/stm32ai-wiki).
  - **Additional Motions**: Jumping, clapping, and swimming datasets collected at **26Hz** using the **I2C feature**, and saved as **CSV files** via the **USB feature**.
- **Low Power Optimization**: Power usage confirmed with a **multimeter**.

## Training the Model
The model is adapted from ST, using **Keras** to process the motion sensing classifier model. For a detailed walkthrough, refer to the [Human Activity Recognition Notebook](https://github.com/STMicroelectronics/stm32ai-wiki/blob/master/AI_resources/HAR/Human_Activity_Recognition.ipynb). Our project introduces additional layers and functions. The trained model is saved in the HDF5 file 'model.h5'.

## Integration of AI Component
Follow these steps to integrate the AI component using [STM32Cube.AI Application](https://wiki.st.com/stm32mcu/wiki/AI:How_to_perform_motion_sensing_on_STM32L4_IoTnode#Add_STM32Cube-AI_to_your_project):
1. Under the artificial intelligence filter, enable core.
2. Configure the X-CUBE-AI component to use the existing Keras model:
   - Click 'ADD Network' -> Set type to 'Keras' -> Browse to select the model.
   - Generate code.
3. Include the CUBE AI header.
4. Declare neural network buffers.
5. Add AI bootstrapping functions.
6. Create an 'argmax' function.
7. Call the `AI_init` function.

## Pin Configuration
### I2C Configuration
- Software Pack -> Select Component -> Board Part -> AccGyr -> LSM6DSL -> I2C
- Pinout & Configuration -> I2C set to 'Fast mode' with speed Frequency '400 kHz'.
- PB10 `I2C2_SCL`
- PB11 `I2C2_SDA`
- PD11 `GPIO_EXIT11`
- Software Packs -> X-Cube_MEMS1 -> Board Part Accgyr -> Platform Setting -> I2C
### UART
- Connectivity -> USART1 -> Mode 'Asynchronous' -> Baud Rate '115200 bit/s'
- Ensure PB6 and PB7 are configured.
### USB Feature 
- PA11 `USB_OTG_FS_DM`
- PA12 `USB_OTG_FS_DP`
- Connectivity -> USB_OTG_FS -> Mode 'Host Only' -> Activate_VBUS 'Disable'
- Middleware and Software Packs -> FATFS -> USB Disk
- Middleware and Software Packs -> USB_HOST -> class 'Mass Storage Host Class' -> Platform Setting -> Drive_VBUS_FS 'GPIO:Output', 'PA9'
## Low Power
To optimize for low power:
- Remove the jumper at pin JP5.
- Use a multimeter to measure the current at the pin.
