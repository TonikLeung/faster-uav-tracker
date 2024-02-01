# faster-uav-tracker
### Develop object tracking algorithm for my drone

### Implemented Tracker: 
LightTrack, more trackers are coming...

# Preparation

Requirements: opencv, tensorrt, realsense2, cuda

run:

```
mkdir build && cd build
cmake ..
make -j8

cd ../workspace
bash compile_engine_lighttrack_z.sh
```
