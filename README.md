# g2o_ba_example
An easy example of doing bundle adjustment within two images using g2o. 

Require: g2o, OpenCV 2.4.x

The program reads two images from the data/1.png and data/2.png, then finds and matches orb key-points between them. After these, it uses g2o to estimate the relative motion between frames and the 3d positions (under a unknown scale).

This is an example written for the beginners of g2o and SLAM.

For more details please see the corresponding blog: http://www.cnblogs.com/gaoxiang12/p/5304272.html (in Chinese only).


## For branch: trans_edge
### for orb_se3_node:
input are world coordinates and image points, output is estimated 6D transformation.

The true points are 7 artifact 3D positions in world coordinate. The measurements are the corresponding 7 image coordinates u,v.

We set the fx=fy=400, cx=230,cy=320. World to camera translation is set to [0, 0, -20], rotation is set to identity matrix.

The cost function is：
```
error = observation - camera_projection(rotation * X_world + translation)
where camera_projection=[X_world.x * fx / X_world.z, X_world.y * fy / X_world.z].
```
So ideally the estimated translation is [0, 0, -20] so that error is minimized.

### for orb_node:
input are world coordinates and image points, output is estimated 3D translation (no rotation as prior).

The true points are 7 artifact 3D positions in world coordinate. The measurements are the corresponding 7 image coordinates u,v.

We set the fx=fy=400, cx=230,cy=320. World to camera translation is set to [0, 0, -20].

The cost function is：
```
error = observation - camera_projection(X_world + translation)

where camera_projection=[X_world.x * fx / X_world.z, X_world.y * fy / X_world.z].
```
So ideally the estimated translation is [0, 0, -20] so that error is minimized.

（However, the orb_node does not work. Now trying to fix it).

