@echo off
.\tools\ST-LINK_Debugger\1.6.0\tools\bin\STM32_Programmer_CLI.exe  -c port=SWD  mode=NORMAL -d rtthread.bin 0x08000000 -s