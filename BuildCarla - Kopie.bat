cd %IQZ_VS_STARTUP%
call vcvars64.bat
cd C:\carla
for %%A in (clean LibCarla PythonAPI launch) do (
    cls
    title make %%A
    make %%A
    
)
choice /C X /N /M "Press X to continue"