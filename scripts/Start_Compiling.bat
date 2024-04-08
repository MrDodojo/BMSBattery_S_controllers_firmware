PATH = %PATH%;C:\Program Files\STMicroelectronics\st_toolset\stvp;C:\Program Files (x86)\STMicroelectronics\st_toolset\stvp;C:\SDCC\usr\local\bin
REM ;%~dp0tools\cygwin\bin
cd %~dp0
sdcc --version
::Move location to project folder
cd ..
del main.hex
del main.ihx
tooling\make -f Makefile_windows clean
tooling\make -f Makefile_windows
ren main.ihx main.hex
tooling\make -f Makefile_windows clean

STVP_CmdLine -BoardName=ST-LINK -ProgMode=SWIM -Port=USB -Device=STM8S105x6 -FileProg=main.hex -verbose -no_loop

pause
exit
