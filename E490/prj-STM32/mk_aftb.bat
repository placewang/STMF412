set yy=%date:~2,2%
set mm=%date:~5,2%
set day=%date:~8,2% 
set YYmmdd=%yy%%mm%%day%
set YYmmdd=%YYmmdd: =%
set a=%YYmmdd%

copy /y E:\work\HJ_Head_E475_STM32F205\prj-STM32\MDK_prj\Objects\E457_STM32F205.hex E:\work\HJ_Head_E475_STM32F205\prj-STM32\E475_%a%.hex

copy /y E:\work\HJ_Head_E475_STM32F205\prj-STM32\Boot_prj\Objects\E490_stm32f205_boot.hex E:\work\HJ_Head_E475_STM32F205\prj-STM32\E490_boot.hex


E:\work\HJ_Head_E475_STM32F205\prj-STM32\hex2pgx.exe --headall --exec-8b 0x8009:E:\work\HJ_Head_E475_STM32F205\prj-STM32\E475_%a%.hex ssss.hex E:\work\HJ_Head_E475_STM32F205\prj-STM32\Head_E490_1s_%a%.pgh


E:\work\HJ_Head_E475_STM32F205\prj-STM32\hex2pgx.exe --headall --exec-8b 0x7009:E:\work\HJ_Head_E475_STM32F205\prj-STM32\E490_boot.hex ssss.hex E:\work\HJ_Head_E475_STM32F205\prj-STM32\Head_E490_boot.pgh
