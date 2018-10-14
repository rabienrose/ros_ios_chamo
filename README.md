# ros_ios_chamo
### The minimal version of ROS compiled to the static lib for ios in order to sent and receive msg between pc and ios

- This version is modified based on the ros indigo. 
- Run build.sh to compile the static libs.
- The output is many static libs include the thridparty lib.
- The dependance are console_bridge and boost.
- console_bridge will be compile automatically in the build.sh.
- You need to find the boost static lib for ios yourself, one of the choice is https://github.com/ziliwangmoe/ofxiOSBoost
- Basically, any version that can generate the static lib of boost and include files is OK.
- Put the boost.a file in the thirdparty/boost/ios, and include file folder (with the name of boost) in thirdparty/boost/include
- The include folder in the project folder is the include directory you need to set to the xcode of your app, and the the lib folder in the project folder is the lib search path you need to provide to xcode.
- For all other advanced msg of ros you want to add, you can generate the msg headers in ubuntu, and copy the .h files to the include folder.
- There is an example of xcode project (data_recorder) show how to use the ros in APP.
- This code is not widely tested and are not targeted to robust, you can understand it and modify based on your situation.
- Use sudo xcode-select -switch /Applications/Xcode.app/ to switch the compiler to xcode
