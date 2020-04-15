@echo off

echo ************************
echo **** Building Laser ****
echo ************************

rd /s /q "build\Visual Studio 16 2019"
cmake.exe -S . -B "build/Visual Studio 16 2019" -G "Visual Studio 16 2019" -A x64
cd build\Visual Studio 16 2019
msbuild /p:Configuration=RelWithDebInfo /t:Build Laser.sln
msbuild /p:Configuration=Release /t:Build Laser.sln
msbuild /p:Configuration=Debug /t:Build Laser.sln
cd ..
cd ..
