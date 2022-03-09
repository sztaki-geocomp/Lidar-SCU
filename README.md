# Point Cloud Registration and Change Detection in Urban Environment Using an Onboard Lidar Sensor and MLS Reference Data
Supplementary material to our submitted paper in the International Journal of Applied Earth Observation and Geoinformation, 2022\
<img src="https://user-images.githubusercontent.com/50795664/156535451-4b0c4332-60d8-4c63-99ca-588564ee873e.png" width="400" height="400"> <img src="https://user-images.githubusercontent.com/50795664/156535768-d5095132-8a04-4578-b65f-b20549c69326.png" width="400" height="400">\
Left: MLS point cloud obtained in Kálvin square, Budapest, Hungary. <br/>
Right: Point cloud frame captured by a Velodyne HDL64E sensor in Kálvin square, Budapest, Hungary. <br/>
## Dataset
The full dataset used in the paper can be downloaded here.
The description of the dataset is available here:
## Citation
If you found this work helpful for your research, or use some part of the code, please cite our submitted paper (currently under review):

```text
@article{lidar-scu, 
	title={{Point Cloud Registration and Change Detection in Urban Environment Using an Onboard Lidar Sensor and MLS Reference Data}}, 
	author={Zováthi, Örkény and Nagy, Balázs and Benedek, Csaba}, 
	journal={International Journal of Applied Earth Observation and Geoinformation}, 
	year={2022},
	note={under review}
}
```

## Installation guide
The code was written in C++ and tested on a desktop computer with Ubuntu 18.04.
Please carefully follow our installation guide.

#### Dependencies:
PCL-1.8.1\
Eigen3\
OpenCV\
CMake

#### Install PCL and Eigen3:
```sh
$ sudo apt install libeigen3-dev libpcl-dev
```
#### Build OpenCV from source:
Please follow the instructions of [this tutorial](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/#installing-opencv-from-the-source).

#### Setting up the project:
1. Clone this repository
2. Build the project:
```sh
$ mkdir build && cd build
$ cmake ..
$ make -j10
```
3. Run the project on sample demo data:
```sh
$ ./Lidar-SCU
```

#### Required data-structure:
|--project_root:\
&nbsp;&nbsp;&nbsp;|--Data:\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--Samples:\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--Velo1.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--MLS1.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--Velo2.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--MLS2.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--Velo3.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--MLS3.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--Velo4.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--MLS4.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--Velo5.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--MLS5.pcd\
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|--Output:

## Simple usage:
Run the algorithms on your own sample demo data pair:
```sh
$ ./Lidar-SCU path_to_rmb_lidar_frame.pcd path_to_mls_data.pcd
```
By default, the execution follows the steps below:
#### 1. Landmark object extraction from dense MLS point clouds

#### 2. Object detection in sparse Lidar scans

#### 3. Bounding-box-based coarse registration

#### 4. Standard ICP alignment

#### 5. Range-image generation

#### 6. MRF-based change detection
For more details, please address our paper.

## Authorship declaration
The code of this repository was implemented in the [Machine Perception Research Laboratory](https://www.sztaki.hu/en/science/departments/mplab), Institute of Computer Science and Control (SZTAKI), Budapest.\
<img src="https://epicinnolabs.hu/wp-content/uploads/2019/10/sztaki_logo_2019_uj_kek.png" width="200">