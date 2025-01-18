# Alex CANBus DevBoard

## Repo:

https://github.com/alexm35644/CANBus_Devkit

## PCB:

[https://kicanvas.org/?github=https%3A%2F%2Fgithub.com%2Falexm35644%2FCANBus_Devkit%2Fblob%2Fmain%2FKiCAD_MCPtest%2FKiCAD_MCPtest.kicad_pro](https://kicanvas.org/?github=https%3A%2F%2Fgithub.com%2Falexm35644%2FCANBus_Devkit%2Fblob%2Fmain%2FKiCAD_MCPtest%2FKiCAD_MCPtest.kicad_pro)

# Summary

This project was made to develop CANBus for UBC ThunderBikes. It features 2 STM32 chips and 2 MCP2562FD CAN transceivers. 

# Firmware

## CAN_Firmware_Left

This firmware is for the chip on the left (near the USB Connector) and it acts as the “Master”. It sends CAN frames every 0.5 seconds. 

## CAN_firmware_Right

This firmware is meant for the chip on the right. It acts as the “Slave”. It receives the CAN frame and prints it via UART.