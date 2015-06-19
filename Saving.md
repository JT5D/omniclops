### Saving images ###

One thing which you might typically wish to do is log images to file periodically.  To have the program save an image and then exit use the following command:

```
    omniclops --dev /dev/video0 --save test.jpg --headless
```

### Saving edge features ###

Alternatively you might wish to only save edge features to file, as follows:

```
    omniclops --dev /dev/video0 --saveedges edges.dat --headless
```

This saves a binary file called edges.dat.  This consists of pairs of unsigned short integers (16 bits x 2) giving the x and y image coordinates of each edge feature.  To try to increase the accuracy the edge values are sub-pixel interpolated, so to obtain a floating point value for the x and y coordinates divide the values by 32.

### Saving 3D rays ###

You might also wish to save the 3D coordinates for rays of light corresponding to observed edge features.  This feature can be useful for applications requiring bundle adjustment or synthesis of the data with that obtained by other sensors.

```
    omniclops --dev /dev/video0 --diam 60 --dist 50 --focal 3.6 --elevation 400 --rays rays.dat --headless
```

The resulting binary file contains 6 signed short integers per ray, specifying the beginning and end of the ray.  The first three values are a point on the mirror surface and the second three values are a point on the ground plane.  All values are in millimetres.