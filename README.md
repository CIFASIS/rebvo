# REBiVO
## Realtime Edge Based Inertial Visual Odometry for a Monocular Camera

Tarrio, J. J., & Pedre, S. (2015). Realtime Edge-Based Visual Odometry
for a Monocular Camera. In Proceedings of the IEEE International Conference
on Computer Vision (pp. 702-710).

Tarrio, J. J., & Pedre, S. (2017). Realtime edge based visual inertial odometry for MAV teleoperation in indoor environments. Journal of Intelligent and Robotic Systems.

REBiVO tracks a camera in Realtime using edges and inertial data, is a slim system targeted for MAV operation. The system is split in 4 components.
A library containing the core (rebvolib/librebvolib.a)
An on-board app (app/rebvorun) to launch the library to do all the processing and send data over UDP
An an OpenGL visualizer (app/visualizer) to show the newwork transmitted data.
A ROS wrapper (under development)

Introductory video: https://youtu.be/7pn29iGklgI

## Original repository
This repository is a modified version of [rebvo](https://github.com/JuanTarrio/rebvo/) (see original README below). We facilitate the installation process and the use of Docker.

## Docker support

In order to facilitate the installation process, the system is wrapped up using Docker.
We provide scripts to create a Docker image, build the system and run it in a Docker container. 

### Dependencies 
* Docker
* ROS
* [`pose_listener`](https://github.com/CIFASIS/pose_listener) (if you use `run_rosario_sequence.sh`, see below)

### Building the system
Run:
```
./run.sh -b
```

This command creates a Docker image, installs all the dependencies and builds the system. The resulting image contains a version of the system ready to be run.

### Running the system in VIS mode
If you are not interested in making changes in the source code, you should run the system in VIS mode. Run:
```
./run.sh -v
```
The system is launched in a Docker container based on the previously built image. By default, this command executes a launch file which is configured to run the Rosario dataset. If you want to run your own dataset, **write a launch file and placed it in** `ros/src/rebvo_ros/launch/`. Configuration files must be placed in the `ros/src/rebvo_ros/config/` folder. Then, run the script with the option `-l <LAUNCH_FILE_NAME>`. For example, if you are testing EuRoC, write `euroc_dataset.launch`, move it into `ros/src/rebvo_ros/launch/` and type:
```
./run.sh -v -l euroc_dataset.launch
```
Making changes in launch/configuration files in the host is possible because these folders are mounted into the Docker container. It is not necessary to access the container through a bash shell to modify these files.

See below for information about input data and visualization.

### Running the system in DEV mode
DEV mode allows developers to make changes in the source code, recompile the system and run it with the modifications. To do this, the whole repository is mounted in a container. Run:
```
./run.sh -d
```
This opens a bash shell in a docker container. You can edit source files in the host and after that you can use this shell to recompile the system. When the compilation process finishes, you can run the method using `roslaunch`.

See below for information about input data and visualization.

### Input data and visualization

At this point, the system is waiting for input data. Either you can run `rosbag play` or you can use `run_rosario_sequence.sh`.
If you choose the latter, open a second terminal and run:
```
./run_rosario_sequence.sh -o <OUTPUT_TRAJECTORY_FILE> <ROSBAG_FILE>
```
In contrast to what `run.sh` does, `run_rosario_sequence.sh` executes commands in the host (you can modify it to use a Docker container). 

`ROSBAG_FILE` is played using `rosbag`. Also, make sure you have cloned and built `pose_listener` in your catkin workspace. Default path for the workspace is `${HOME}/catkin_ws`, set `CATKIN_WS_DIR` if the workspace is somewhere else (e.g.: `export CATKIN_WS_DIR=$HOME/foo_catkin_ws`). `pose_listener` saves the estimated trajectory in `<OUTPUT_TRAJECTORY_FILE>` (use absolute path). You can edit `run_rosario_sequence.sh` if you prefer to save the trajectory using your own methods. Additionally, `run_rosario_sequence.sh` launches `rviz` to display visual information during the execution of the system.

Alternatively, if you are not interested in development but in testing or visualization, instead of running `run.sh` and `run_rosario_sequence.sh` in two different terminals, you can just run:
```
./run_rosario_sequence.sh -r -o <OUTPUT_TRAJECTORY_FILE> <ROSBAG_FILE>
```
This launches a Docker container and executes the default launch file (see `LAUNCH_FILE` in `run.sh`). After that, the bagfile is played and `rviz` and `pose_listener` are launched. Add `-b` if you want to turn off the visualization.



# Original README

### System requirements

In ubuntu and most linux dist this libraries can be downloaded directly from the
 repos, except for TooN.

-- C++11

-- Linux, X11, v4l2

-- OpenGL development libraries (GL,GLU,glut)

-- TooN 2.2 mathematical library (http://www.edwardrosten.com/cvd/toon.html - ZIP provided in the repo)

-- Lapack (for advanced TooN functions)

-- LibAV (Video Codecs)

-- LibGD (Image managment)

-- Optionally NE10 for ARM Neon optimizations

### Compiling

REBVO has been developed using QT creator, so a project file is provided in the main folder. Also a makefile is provided on the main directory for 64bit machines.

For x86 compile using (on the root directory):

make

For 64bit desktop use:

make REBVOFLAGS=-m64

for ARM:

make REBVOFLAGS='-mtune=cortex-a15 -mfpu=neon'


for ARM using NE10

make REBVOFLAGS='-mtune=cortex-a15 -mfpu=neon -DUSE_NE10 -lNE10'

#### Output

Library: rebvolib/librebvolib.a
Rebvo Launcher: app/rebvorun/rebvorun
Visualizer: app/visualizer/visualizer

### Imu Integration

IMU measurements are integrated trough the ImuGrabber class, that implements a circular buffer, timestamp search utilities and inter-frame integration.

For IMU data in a csv dataset style the ImuGraber supports a file loadding function (Config IMUMode=2).
For a custom IMU, timestamped data can be pushed using REBVO::pushIMU() function.

#### Imu Fusion

IMU fusion is done using a two stage Bayesian filter. Sensor noise covariances should be set for optimal performance.

An initial guess for Giro Bias should be provided for highly biased sensors, an initial automatic guess could be used if the system is started still.

### Camera Drivers

Three classes are provided for camera management:

-- v4lCam is a wrapper to the C functions provided in video_io for interacting with
   the v4l2 lib.

-- SimCamera is a simple class designed to load uncompressed video from a file. For compressed video
   formats check the Video2SimCam section.

-- DataSetCam is used to load the images from the TUM datasets used to benchmark the
   paper (add the dataset directory and image file list to the config file).

-- Custom cam, a class for external loading of images

All three classes inherit from VideoCam class, this class is able to generate the
video files used by simcam.


#### Using the custom camera

If the custom camera is selected, images has to be loaded to the rebvo object. Use the REBVO:requestCustomCamBuffer()
function to request an Image buffer, save your image to that buffer and call REBVO::releaseCustomCamBuffer to release.
This can be done in the aplication thread, images are passed transparently using a pipe circular buffer.

#### Video2SimCam Utility

Currently rebvo canot load compressed video directly (a feature that is gonna be added soon), so
a simple utility is provided in the Video2SimCam folder that uses OpenCV VideoCapture to uncompress
the video in the SimCam format (can take a lot of disk space!).

### Customizing the Output

A callback function can be configured using the library function  REBVO::setOutputCallback(). This a callback is called on the third thread with a reference
to a struct containing all the algorithm's output.

The funtion REBVO::getNav() can also bue used to extract navigation parameters

### Ros Support

The folder ros/src/rebvo_ros contains an ROS-indigo wrapper that is under development. 2 launchers and configuration files are provided, one for monocular camera
and the second for trying on the EuRoMav Dataset using ros bag files. The nodelet is algo a good example of how to use the library.

### Configuring the system

A GlobalConfig text file can be found in each of the folders. This should be tuned to
your camera and network configuration. All the configuration options has to be present on the files
in order for the system to start. The structure of the file is self explanatory.

Two files are provided as an example, GlobalConfig (and standart config to run onboard Quad) and GlobalConfig_EuRoC (for testing the EuRoC dataset).

#### Basic configs in rebvo GlobalConfig

The things you have to configure in order for the system to work.

-- CameraDevice: the v4l2 camera device or dataset camera file and directory

-- ZfX,Y: Camera focal length

-- PPx,y: Camera principal point

-- ImageWidth,Height: size to use

-- Distortion parameters (if enabled)

-- FPS: Frames per Seconds

-- VideoNetHost,Port: IP,Port of the host to send data to

-- SetAffinity flag,CamaraTn: if the flag is set to 1, the system will use the pthread_setaffinity_np() call
		to distribute the threads according to the CamaraT1..3 configuration.
		This numbers have to point to existing processors or the call wil fail,
		and this setting usually require superuser privileges.
		Setting affinity and distributing the threads can increase speed.

-- DetectorThresh: This is a fixed threshold to the edge detector only used
	 	   if autothresh is off (DetectorAutoGain = 0)
		   Is a camera dependent parameter so it should be tunned for
		   best performance. 

-- ReferencePoints: If auto thresh is on (DetectorAutoGain>0), number of KL
		   points to use. Ussual values are 5000 for 320x240 and 
		   15000 for 640x480 

-- DetectorMax,MinThresh: Limits on threshold tunning, to prevent excessive 
		   noise on images with no edges. Similar to DetectorThresh,
		   camera dependant parameters.

-- Imu Mode: 1 for custom IMU, 2 for dataset IMU (EuRoC MAV format)

-- Imu-Camera RotoTraslation file

-- IMU Noise parameters

#### Basic configs in visualizer GlobalConfig

ImageSize, Focal Length and Principal Point should match the config in rebvo.

### System Usage

Just run the two programs on the same o network connected computers and move
the camera trying to maximize translation.

REBVORUN component accept the following command line commands:

q: Quit

r: System full reset

s: Saves in SimVideoFile (rebvo GlobalConfig) SimVideoNFrames frames of
   uncompressed video for further replay. It does a direct write to disk,
   so it can loose frames depending on the system. The format is described
   in videocam.h

p: Takes Snapshot of the current image


The visualizer opens 2 kinds of windows, XLib windows (front and top) 
displays the image and the edge detection, OpenGL windows display 
the edgemap and the image in a 3D fashion.

#### Commands in the XLib windows

q: Quit

b: Clear trajectory

i: Toggle show image

t: Show frame to frame matching

z,x: Increase/Decease maximum depth in top view

#### Commands in the OpenGL windows

w,s,a,d: Walk the view in a DukeNukem style
arrows: Rotate the view

f,r: flight

1..9: Load i view

Shift+1..9: Save i view

Escape: Reset View

m: Toggle image rendering

n: Toggle edgemap rendering

c: Toggle camera rendering

t,g: Increase/decrease max distance for color scale

y,h: Same for min distance 

i: Render edgemap depth uncertainty

v: Render Trajectory

b: Clear trajectory

,: Fix the view to world coordinates

End: Quit

### Output files

If the savelog option is enabled the system outputs 2 files, a .m log file and a
trajectory file (timestamp tx ty tz qx qy qz qw) that can be used to benchmark
the algorithm.

### FAQ


-- Why rebvo doesn't use OpenCV?

OpenCV is great, the system originally used OpenCV for image acquisition 
and preprocessing. 
Then at some point we figure that it was a waste having to install
the whole library just for image acquisition, so I write small a wrapper 
to the V4L2 library trying to make the system easier to install and 
less dependent on third party libraries.


### Contact

Juan Jose Tarrio

juan.tarrio@gmail.com
