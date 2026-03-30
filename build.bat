@echo off
"C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" "C:\Users\stu12\RTCK_DEV\OpenVR-SpaceCalibrator-1\OpenVR-SpaceCalibrator.sln" /p:Configuration=Release /p:Platform=x64 /m /v:normal
echo EXIT_CODE=%ERRORLEVEL%
