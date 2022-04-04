# SI-VSLAM
This is a universal semantic slam system for RGB-D camera based on ORB-SLAM2 and DS-SLAM (from https://github.com/raulmur/ORB_SLAM2, https://github.com/ivipsourcecode/DS-SLAM and https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map, thanks for Raul's, Gao Xiang's and Chao Yu's great work). The program has three inputs, corresponding to RGB sequence, depth sequence and semantic sequence.

You can watch this conference paper presentation on Youtube to see more of our work: https://www.youtube.com/watch?v=EOlJpMNYCNg

![Flowchart of the SI-VSLAM architecture](https://github.com/mjtq-slamlearning/SI-VSLAM/blob/master/media/SI_VSLAM_Architecture.jpg?raw=true)

Dynamic objects can be recognized by an additional neural network which can work independently from SI-VSLAM. Based on the segmented images, these dynamic objects are removed from the tracking thread and thus do not violate the static scene assumption. The program can selectively output semantic or RGB point clouds. The semantic octomap can only be used in the ROS environment and viewed through Rviz.

# License
This implementation is based on ORB-SLAM2, which is licensed under the GNU GENERAL PUBLIC LICENSE Version 3 (GPLv3). Therefore we have to provide our modifications to the code under the same license.

# Prerequisites
We have tested the library in Ubuntu 16.04 and 18.04. The experiment is performed on a computer with Intel i7 CPU, GTX 1050Ti GPU and 20GB RAM. To add a neural network module to the system, we recommend using a more powerful graphics card.

## ORB_SLAM2 Prereguisities
SI-VSLAM is a semantic SLAM system based on ORB-SLAM2. In order to run SI-VSLAM, you have to install environment needed by ORB-SLAM2 (the section of 2. Prereguisites). The Bag of Words model can be downloaded from https://drive.google.com/file/d/1Q9idM_UqQjr032okKWlEkp4ZAfvy8MgR/view?usp=sharing, please place it in the Vocabulary folder.

## ROS(optional)
We provide some examples to process the live input of a RGB-D camera using ROS. Building these examples is optional. The ROS version we are using is melodic.

# How to install
Clone the repository:
```
git clone https://github.com/mjtq-slamlearning/SI-VSLAM.git
```
We will refer to the path where the project was cloned to as `{PATH_SIVSLAM}` 

Open file path：
```
cd {PATH_SIVSLAM}/aa
```
Configuring and building Thirdparty/DBoW2 ...:
```
cd ORBSLAM2/ORB_SLAM2_modified/Thirdparty/DBoW2
mkdir build
cd build
cmake ..
make -j8
```
Configuring and building Thirdparty/g2o ...:
```
cd ../../../../g2o_with_orbslam2
mkdir build
cd build
cmake ..
make -j8
sudo make install
```
Configuring and building ORB_SLAM2 ...:
```
cd ../../ORB_SLAM2_modified/
mkdir build
cd build
cmake ..
make -j8
```
This has completed the basic module of the program, you can already view the generated RGB point cloud and semantic point cloud through **pcl_viewer**.
# Use ROS and view the octree map through Rviz （optional）
Building ROS nodes:
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:{PATH_SIVSLAM}/aa/ORBSLAM2/ORB_SLAM2_modified/Examples/ROS
cd ../Examples/ROS/ORB_SLAM2/
mkdir build
cd build
cmake ..
make -j8
```
octomapserver:
```
cd ../../../../../../catkin_octo/
catkin_make 
source {PATH_SIVSLAM}/aa/catkin_octo/devel/setup.bash ##Change according to own requirements
```
## Dataset for semantic visual SLAM
The version of ROS we are using is melodic. Considering that existing public data sets, such as TUM Benchmark, do not provide semantic image sequences, we used 3D scene simulation software Blender to make a semantic SLAM data set for testing. You can get perfect rgb images, depth images and semantic images through the Vision-Blender plugin (https://github.com/Cartucho/vision_blender, thanks for Cartucho's great work). You can download the data set from https://drive.google.com/file/d/1hAvfanOrBfE3KJph_J1dfe9gon1lkdHl/view?usp=sharing. You can also use your own data set, the original data format is 0001, 0002...，for data sets in other formats, please modify accordingly.
## Run the Programm
```
cd ORBSLAM2/ORB_SLAM2_modified/Examples/ROS/ORB_SLAM2
roscore
sh run.sh
rviz
```
//add topic pointcloud2
octomap:
```
cd aa/catkin_octo/src/
roslaunch octomap_server octomap_mapping.launch
```
//add ColorOccupancyGrid
//Octomap Topic /octomap_full

We have also uploaded a video which demonstrates how to compile and run SI-VSLAM both with and without ROS:
https://github.com/mjtq-slamlearning/SI-VSLAM/blob/master/media/SI_SLAM_withoutros+ros.mp4
