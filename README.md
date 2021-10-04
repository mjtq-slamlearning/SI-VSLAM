# SI-VSLAM
This is a universal semantic slam system for RGB-D camera based on ORB-SLAM2 and DS-SLAM (from https://github.com/raulmur/ORB_SLAM2, https://github.com/ivipsourcecode/DS-SLAM and https://github.com/gaoxiang12/ORBSLAM2_with_pointcloud_map, thanks for Raul's, Gao Xiang's and Chao Yu's great work). The program has three inputs, corresponding to RGB sequence, depth sequence and semantic sequence. Dynamic objects can be recognized by an additional neural network and removed from the image based on semantic segmentation. The program can selectively output semantic point cloud or RGB point cloud. The semantic octomap can only be used in the ROS environment and can be viewed through Rviz.
# License
# Prerequisites
We have tested the library in Ubuntu 16.04 and 18.04. The experiment is performed on a computer with Intel i7 CPU, GTX 1050Ti GPU and 20GB RAM. To add a neural network module to the system, we recommend using a more powerful graphics card.
## ORB_SLAM2 Prereguisities
SI-VSLAM is a semantic SLAM system based on ORB-SLAM2. In order to run SI-VSLAM, you have to install environment needed by ORB-SLAM2 (the section of 2. Prereguisites). The Bag of Words model can be downloaded from https://drive.google.com/file/d/1Q9idM_UqQjr032okKWlEkp4ZAfvy8MgR/view?usp=sharing, please put in the Vocabulary folder.

## ROS(optional)
We provide some examples to process the live input of a RGB-D camera using ROS. Building these examples is optional. The ROS version we are using is melodic.
# How to install
Open file path：
```
cd aa
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
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/sun/aa/ORBSLAM2/ORB_SLAM2_modified/Examples/ROS
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
source /home/sun/aa/catkin_octo/devel/setup.bash ##here muss selbst machen
```
## Dataset for semantic visual SLAM
The version of ROS we are using is melodic. Considering that existing public data sets, such as TUM Benchmark, do not provide semantic image sequences, we used 3D scene simulation software Blender to make a semantic SLAM data set for testing. You can get perfect rgb images, depth images and semantic images through the Vision-Blender plugin (https://github.com/Cartucho/vision_blender, thanks for Cartucho's great work). This implementation is based on ORB-SLAM2, which is licensed under the GNU GENERAL PUBLIC LICENSE Version 3 (GPLv3). Therefore we have to provide our modifications to the code under the same license. You can download the data set from https://drive.google.com/file/d/1hAvfanOrBfE3KJph_J1dfe9gon1lkdHl/view?usp=sharing. You can also use your own data set, the original data format is 0001, 0002...，for data sets in other formats, please modify accordingly.
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
## add ColorOccupancyGrid
## Octomap Topic /octomap_full


