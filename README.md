# Camera Positioner


This package provides a continuous tf transform of a (movable) camera based on Apriltags detected in the image.

---

## Usage

- The camera positioner can be started with the following command:

```
roslaunch camera_positioner camera_positioner.launch
```

- This will not start any camera drivers, it will just publish the transform from the world frame to the camera's root frame and update it everytime the apriltag is detected.

## Parameters:

- `camera_rgb_optical_frame` the frame that used for detect april tag
- `camera_link` Base camera link
- `shared_frame` Shared frame
- `world_frame` World frame
- `static_camera` Weather the camera is static
