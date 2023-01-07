set yy=%date:~2,2%
set mm=%date:~5,2%
set day=%date:~8,2% 
set YYmmdd=%yy%%mm%%day%
set YYmmdd=%YYmmdd: =%
set a=%YYmmdd%

copy /y C:\Users\hqwdc\Desktop\E845\E845V0.0\E845V0.0\MDK-ARM\E845V0.0\E845V0.hex .\E845_%a%.hex

C:\Users\hqwdc\Desktop\E845\E845V0.0\PGH_TOOLS\tools\hex2pgx.exe --headall --exec-8b 0x8009:E845_%a%.hex sss.hex E845_%a%.pgh