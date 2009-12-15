
all:
	g++ -O3 -o omniclops main.cpp anyoption.cpp linefit.cpp drawing.cpp libcam.cpp fast.cpp omni.cpp polynomial.cpp -I/usr/local/include/opencv `pkg-config --cflags --libs gstreamer-0.10` -L/usr/local/lib -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10
 
debug:
	g++ -g -o omniclops main.cpp anyoption.cpp linefit.cpp drawing.cpp libcam.cpp fast.cpp omni.cpp polynomial.cpp -I/usr/local/include/opencv `pkg-config --cflags --libs gstreamer-0.10` -L/usr/local/lib -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10

clean:
	rm -f omniclops




