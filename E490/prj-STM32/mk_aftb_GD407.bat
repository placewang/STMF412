set yy=%date:~2,2%
set mm=%date:~5,2%
set day=%date:~8,2% 
set YYmmdd=%yy%%mm%%day%
set YYmmdd=%YYmmdd: =%
set a=%YYmmdd%

copy /y E:\work\HJ_Head_E475_STM32F205\prj-STM32\MDK_prj\Objects\E490_GD32F407.hex E:\work\HJ_Head_E475_STM32F205\prj-STM32\E490_GD32F407_%a%.hex


E:\work\HJ_Head_E475_STM32F205\prj-STM32\hex2pgx.exe --headall --exec-8b 0x8009:E:\work\HJ_Head_E475_STM32F205\prj-STM32\E490_GD32F407_%a%.hex ssss.hex E:\work\HJ_Head_E475_STM32F205\prj-STM32\Head_E490GD407_1s_%a%.pgh


