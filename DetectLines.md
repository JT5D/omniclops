Detecting lines within the image can be useful for localisation or visual odometry.

Example usage:

```
    omniclops --dev /dev/video0 --lines
```

Optionally lines can be saved to file for subsequent analysis.

```
    omniclops --dev /dev/video0 --saveradial lines.dat
```

The file contains 5 integer values per entry:

  1. Orientation of the line in degrees
  1. start x coordinate in pixels
  1. start y coordinate in pixels
  1. end x coordinate in pixels
  1. end y coordinate in pixels