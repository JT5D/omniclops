A useful feature is to be able to project the image from a spherical mirror onto the ground plane.  This requires that the system know a few things about its geometry.

Example usage:

```
    omniclops --dev /dev/video0 --diam 60 --dist 50 --focal 3.6 --elevation 400 --ground
```

where _diam_ is the diameter of the spherical mirror, _dist_ is the distance from the camera lens to the closest point of the mirror, _elevation_ is the typical height of the camera above the ground plane and _focal_ is the camera focal length.  All distances are in millimetres.

Planar projection helps to remove the high degree of distortion around the periphery, and this results in the blocky appearance of the below image.  Such images, although of low quality, can be useful for visual odometry purposes, since straight lines in the real world appear (mostly) straight within the image.

![http://lh6.ggpht.com/_cGREIsCvj4M/SyTEBQJwaWI/AAAAAAAAAXA/uJeN4ETXBgM/groundplane.png](http://lh6.ggpht.com/_cGREIsCvj4M/SyTEBQJwaWI/AAAAAAAAAXA/uJeN4ETXBgM/groundplane.png)

If you wish to create an image showing the paths of rays of light for your specified geometry, use something like the following:

```
    omniclops --dev /dev/video0 --diam 60 --dist 50 --focal 3.6 --elevation 400 --raypaths rays.jpg
```

This will save an image called _raypaths.jpg_.  At present only spherical mirrors are supported, although the system could be adapted for other types of mirror geometry.

![http://lh4.ggpht.com/_cGREIsCvj4M/SyTBC5pILyI/AAAAAAAAAW4/_xeJZCKRjdY/raypaths.jpg](http://lh4.ggpht.com/_cGREIsCvj4M/SyTBC5pILyI/AAAAAAAAAW4/_xeJZCKRjdY/raypaths.jpg)