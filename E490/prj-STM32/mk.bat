copy /y .\MDK_prj\Objects\E457_STM32F205.hex .\E475.hex


hex2pgx.exe --headall --exec-8b 0x8007:E475.hex ssss.hex Head_E475_1s.pgh

hex2pgx.exe --headall --exec-8b 0x8008:E475.hex ssss.hex Head_E480_1s.pgh

hex2pgx.exe --headall --exec-8b 0x8009:E475.hex ssss.hex Head_E490_1s.pgh