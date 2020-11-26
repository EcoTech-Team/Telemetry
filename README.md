# Telemetry

This project was a part of the electronic system of hydro-powered vehicles. Telemetry
module provides data collecting from RS-485 bus and store it in a flash drive.
Here is a code for STM32F103C8T6 microcontroller.

## IDE configuration
This project was created in Visual Studio Code with `stm32-for-vscode` extension
which requires some other programs and extensions to work properly:
- `STM32 CubeMX` for microcontroller configuration and code generation (HAL
Libraries)
- `Cortex-Debug` extension
- `ST-Link (Drivers and Utility)`
- `GNU Arm Embedded Toolchain`

For more information please refer to the extension site.

## MCU Configuration
For this purpose `STM32 CubeMX` was used. [Here](https://www.youtube.com/watch?v=szMGedsp9jc) you can find a tutorial with a quick start.

## Data analysis

For this purpose python script `analyze_data` was created. It require to run
it with one additional parameter which provide a path to the location of telemetry
folders `MotorDriver` and `MotorController`:
```bash
$ python3 analyze_data.py /mnt/Telemetry
```
> Note: Telemetry folder includes `MotorDriver` and `MotorController` folders

Before run the script installing some packages is necessary. You can do it by
running this command:
```python
$ pip3 install -r requirements.txt
```
