# Tools — hardware-free pipeline self-test

`mapping_sim.cpp` runs the **entire mapping pipeline without any hardware**. It
synthesises a structured scene, flies a platform through it, generates the three
raw sensor streams (per-frame LiDAR, 200 Hz IMU attitude, 20 Hz UM982 RTK
position) and then runs the *exact* reconstruction maths from the ROS2 node
(Mercator georeferencing, SLERP attitude interpolation, linear RTK
interpolation, per-point deskew, single global anchor).

It writes a reconstructed georeferenced point cloud and reports the
reconstruction error against ground truth. This is useful to:

- see real point-cloud output before the sensors/mini-PC are ready;
- validate a build end-to-end (the global anchor, the projection, the deskew);
- regression-test the maths after any change.

## Run

```bash
# Eigen headers required (apt install libeigen3-dev, or a local copy)
g++ -std=c++17 -I /usr/include/eigen3 tools/mapping_sim.cpp -o /tmp/mapping_sim
/tmp/mapping_sim          # writes /tmp/sim_map.pcd and /tmp/sim_map.xyzrgb

# render to a PNG (matplotlib)
python3 tools/render_cloud.py
```

## Expected result

```
frames:        80
mapped points: 376834
recon error:   max ~0.05 mm, mean ~0.02 mm
```

The residual (~0.05 mm) is purely the pose-interpolation error between the
200 Hz IMU / 20 Hz RTK samples — proof the frames stitch into one consistent
world map. A per-frame anchor (the bug that was fixed in the port) instead
produces metre-scale errors because each frame collapses onto its own origin.

A rendered example is in `sample_output/sim_map.png`; the full cloud is in
`sample_output/sim_map.pcd.gz` (open with CloudCompare or `pcl_viewer`).
