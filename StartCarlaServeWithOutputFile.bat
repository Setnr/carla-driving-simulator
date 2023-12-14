cd %IQZ_VS_STARTUP%
call vcvars64.bat
cd C:\carla
del BuildCarlaOutput.set
make launch >> BuildCarlaOutput.set
pause

