It's possible to stream video over a network using [gstreamer](http://gstreamer.freedesktop.org/).  This can be useful for remotely operated vehicles or surveillance systems.

On the server:

```
    omniclops --dev /dev/video0 --stream --headless
```

then on the client:

```
    gst-launch tcpclientsrc host=[ip] port=5000 ! multipartdemux ! jpegdec ! autovideosink
```

To do a local test you can use _localhost_ as the IP address.

![http://lh5.ggpht.com/_cGREIsCvj4M/SyTUq0d17UI/AAAAAAAAAXI/6E7SetNM2Ts/s640/videostreaming.png](http://lh5.ggpht.com/_cGREIsCvj4M/SyTUq0d17UI/AAAAAAAAAXI/6E7SetNM2Ts/s640/videostreaming.png)