# Lidar obstacle detection project

### Description.

The goal of this project is to implement obstacle detection with data comming from a lidar Point cloud.
The inner algorithms have been implemented from scratch except for point cloud formating and rendering where I used PCL library.
This is a educational project to understand the main algorithms and learn the use of PCL library

### Plane segmentation
<img src="media/plane_obst_segment.jpg" width="777" height="455" />

### Obstacle clustering
<img src="media/obst_clustering.jpg" width="895" height="320" />

### Obstacle clustering with bounding boxes
<img src="media/obst_clustering2.jpg" width="655" height="375" />


### Requirements and installation

You need to install PCL library on your system.

```bash
$> sudo apt install libpcl-dev
$> sudo apt install libproj-dev
```

### Now clone this repository:

```bash
$> git clone https://github.com/eslavaj/lidar_obst_detect.git
```

### To build
```bash
$> cd lidar_obst_detect
$> mkdir build && cd build
$> cmake ..
$> make
```

### To run the application
```bash
$> cd build
$> ./obst_detection
```


