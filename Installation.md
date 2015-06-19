To install the software first you will need to ensure that OpenCV version 2 is installed.  A package for this can be downloaded from the project home page or downloads tab.  At the time of writing this is not the standard version of the OpenCV libraries available from within the package manager, and the main advantage is that the newer version allows images to be encoded for video streaming.

A Debian type Linux operating system is assumed here.  To install from the terminal type:

```
    sudo dpkg -i opencv-2.0.deb
```

Then the main utility can be installed as follows:

```
    sudo dpkg -i omniclops-x.xx.deb
```

Any additional dependencies should be installed automatically.

Alternatively, once the OpenCV libraries have been installed you can also compile from source in the usual way with:

```
    make clean
    make
```

If you make changes to the code you can also easily create a new deb package by running:

```
    sh makepackage.sh
```


**Configuring for use with Eclipse**

To use with the Eclipse C++ IDE:

  * Create a new C++ project

  * Import the source from the _Import/File System option_

  * Within the project properties select _C++ Build/Settings_

  * Under _GCC C++ Compiler/Directories/Include paths (-l)_ enter

```
/usr/include/opencv
/usr/include/gstreamer-0.10
```

  * Under _GCC C++ Compiler/Miscellaneous_ enter

```
-c -fmessage-length=0 -lcam -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs gstreamer-0.10` -L/usr/lib -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10
```

  * Under _GCC C++ Linker/Libraries/Libraries (-l)_ enter

```
cv
cxcore
gstapp-0.10
highgui
```

  * Under _GCC C++ Linker/Libraries/Library search path (-L)_ enter

```
/usr/lib
/usr/lib/gstreamer-0.10
```

  * Under _GCC C++ Linker/Miscellaneous/Linker flags_ enter

```
/usr/lib/libcxcore.so /usr/lib/libcvaux.so /usr/lib/libcv.so /usr/lib/libhighgui.so
```

After applying those settings you should now be able to compile the project within Eclipse.