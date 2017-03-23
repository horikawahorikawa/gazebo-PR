# How to Build gazebo-6  

## 1.Install OculusVR SDK  

In case ouclus version 0.4.4  
~~~bash
$ sudo apt-get install libusb-dev libudev-dev libxinerama-dev
$ hg clone https://bitbucket.org/osrf/oculussdk
$ cd oculussdk
$ mkdir build
$ cd build
~~~  

If Oculus SDK Version is 0.4.4, rewrite "add_libarary(ovr ~~)" to "add_libarary(ovr SHARED ~~)" in /LibOVR/CMakeLists.txt  
~~~bash
$ cmake .. -DCMAKE_INSTALL_PREFIX=/usr
$ make
$ sudo make install
$ sudo cp ../LibOVR/90-oculus.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
~~~  

## 2.Build Gazebo from source  

Install the following  
\*\*Confirm your gazebo and suitable the following libraries version. You should see ./edit_files/Gazebo_depedencies_memo.md.

```shell
libsdformat libignition_math libogre
```  

Add path Oculus Library, Include in gazebo/rendering/CMakeLists.txt  

Build  
~~~bash
$ cmake
$ make
$ sudo make insatall
~~~  

If you decide to install gazebo in a local directory you'll need to modify some of your PATHs:  
~~~bash
$ echo "export LD_LIBRARY_PATH=<install_path>/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
$ echo "export PATH=<install_path>/local/bin:$PATH" >> ~/.bashrc
$ echo "export PKG_CONFIG_PATH=<install_path>/local/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
$ source ~/.bashrc
~~~  

Now try running gazebo:  
If Gazebo runs successfully, you're done!.  

If Gazebo was installed to /usr/local/ and running gazebo throws an error similar to:  
gazebo: error while loading shared libraries: libgazebo_common.so.1: cannot open shared object file: No such file or directory  
, then /usr/local/lib is not in load path (default behavior for Ubuntu). Run the following commands and then try running gazebo again:  
~~~bash
$ echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
$ sudo ldconfig
~~~  

## 3.After compiling Gazebo, If the following isn't written in ~\.gazebo\gui.ini, you add the following.  
### gui.ini  
```
[oculus]
x=<REPLACE_BY_YOUR_HORIZONTAL_RESOLUTION>
y=0
visual=<REPLACE_BY_THE_VISUAL_LINK_ATTACHED_TO_OCULUS>
autolaunch=0
```  

## 4.Run oculus daemon  
```shell
$ ./oculusd
```  

## 5.Run gazebo  
```shell
$ gazebo
```  

Click Oculus Rift in Toolbar
