@echo off

echo ************************
echo **** Building Laser ****
echo ************************

rd /s /q "build\Visual Studio 14 2015"
cmake.exe -S . -B "build/Visual Studio 14 2015" -G "Visual Studio 14 2015" -A x64
cd build\Visual Studio 14 2015
msbuild /p:Configuration=RelWithDebInfo /t:Build Laser.sln
msbuild /p:Configuration=Release /t:Build Laser.sln
msbuild /p:Configuration=Debug /t:Build Laser.sln
cd ..
cd ..
