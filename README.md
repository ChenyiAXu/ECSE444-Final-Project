# 444 Final Project: Human Motion Detection (HMR)

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
  - 'ADD Network' --> type to 'Keras' --> browse to select model
  - generate code
- Include CUBE AI header
- Declare neural network buffers
- Add AI bootstrapping functions
- create an 'argmax' function
- Call the AI_init function
  
## USB Feature 


## Lower Power
