
all:
	g++ -O3 -o omniclops main.cpp anyoption.cpp drawing.cpp libcam.cpp fast.cpp omni.cpp harris.cpp motion.cpp polynomial.cpp -I/usr/include/opencv `pkg-config --cflags --libs gstreamer-0.10` -L/usr/lib -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10 -fopenmp
 
debug:
	g++ -g -o omniclops main.cpp anyoption.cpp drawing.cpp libcam.cpp fast.cpp omni.cpp harris.cpp motion.cpp polynomial.cpp -I/usr/include/opencv `pkg-config --cflags --libs gstreamer-0.10` -L/usr/lib -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10 -fopenmp

clean:
	rm -f omniclops

