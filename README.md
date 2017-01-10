camera_positioner
======

This repository provides the positioning of a camera in tams' lab setup via apriltags.

---

__Usage__

The camera positioner can be started with the following command:

```roslaunch camera_positioner camera_positioner.launch```

This will not start any camera drivers, this will just publish
the transform from the world to the camera and updates it everytime the aprtiltag is detected.
