camera_positioner
======

This package provides a continuous tf transform of a (movable) camera based on Apriltags detected in the image.

---

__Usage__

The camera positioner can be started with the following command:

```
roslaunch camera_positioner camera_positioner.launch
```

This will not start any camera drivers, it will just publish the transform from the world frame to the camera's root frame and update it everytime the apriltag is detected.
