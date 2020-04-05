@echo off

echo ************************
echo **** Building Laser ****
echo ************************

rd /s /q "build\Visual Studio 15 2017"
cmake.exe -S . -B "build/Visual Studio 15 2017" -G "Visual Studio 15 2017" -A x64
cd build\Visual Studio 15 2017
msbuild /p:Configuration=RelWithDebInfo /t:Build Laser.sln
msbuild /p:Configuration=Release /t:Build Laser.sln
msbuild /p:Configuration=Debug /t:Build Laser.sln
cd ..
cd ..
