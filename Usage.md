The most basic way of running the software is as follows:

```
    omniclops --dev /dev/video0
```

where video0 is the camera device to be used.  To find out what video devices are available just type:

```
    ls /dev/video*
```

One of the first things which you'll need to do is specify the inner and outer radii for the mirror.  These are expressed as percentages.

In this example the outer radius is set to 90% and the inner to 15%.  There are no limits on these figures, so potentially the outer radius could be set to greater than 100% if the camera is close to the mirror.

```
    omniclops --dev /dev/video0 --radius 90 --inner 15
```

The purpose of the inner radius is to remove the camera itself from the image, since this is typically not of interest.  The inner radius can be set to zero if you wish to include the camera.

To exit the program press the Escape key.