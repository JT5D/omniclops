When running the software on embedded systems, or if you wish to save a single image and then exit, it can be useful to run without displaying the image on the screen.

Headless mode can be enabled as follows:

```
    omniclops --dev /dev/video0 --save test.jpg --headless
```