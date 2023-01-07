cp prj-STM32/main-1s.hex stm32-1s.hex
cp prj-STM32/main-2s.hex stm32-2s.hex
cp prj-LPC2919/debug/HKhead.hex 2919.hex
cp prj-TMS320LF2407/head.hex 2407.hex

hex2pgx_V6.exe --headall \
--exec-8b 32773:stm32-1s.hex \
--exec-8b 32772:stm32-2s.hex \
--exec-8b 32770:2919.hex \
--exec 32769:2407.hex \
dummy-dummy.hex headV$1.pgh

