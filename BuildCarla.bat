cd %IQZ_VS_STARTUP%
call vcvars64.bat
cd C:\carla
make PythonAPI
pause
make LibCarla
pause
make launch
pause
