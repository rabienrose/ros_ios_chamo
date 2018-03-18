mkdir thirdparty/console_bridge/build
cd thirdparty/console_bridge/build
cmake -DIOS_PLATFORM=OS -DCMAKE_TOOLCHAIN_FILE=../../../iOS.cmake -GXcode ..
xcodebuild -configuration Release -target ALL_BUILD
cd ../../../
mkdir build
cd build
cmake -DIOS_PLATFORM=OS -DCMAKE_TOOLCHAIN_FILE=../iOS.cmake -GXcode ..
xcodebuild -configuration Release -target ALL_BUILD
cd ..
bash ./movelib.sh
