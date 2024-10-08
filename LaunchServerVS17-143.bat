cd %IQZ_VS_STARTUP%
call vcvars64.bat
cd C:\carla
make launch GENERATOR="Visual Studio 17 2022"
pause