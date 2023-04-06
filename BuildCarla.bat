cd %IQZ_VS_STARTUP%
call vcvars64.bat
cd C:\carla
make PythonAPI
make launch
