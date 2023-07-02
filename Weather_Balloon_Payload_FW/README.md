# Payload Firmware

The boilerplate code was generated using STM32CubeMX. 

You can build using make, and it's convenient to use Visual Studio Code with the stm32-for-vscode extension.

## SD Card Driver

Big thanks to projects by kiwih for providing the boilerplate [SD card SPI driver code](https://01001000.xyz/2020-08-09-Tutorial-STM32CubeIDE-SD-card/).

The FAT filesystem middleware was provided by ST via STM32CubeMX.

You can configure the radio by including a file named `config.txt` on the SD card. I've included an example of such a file.