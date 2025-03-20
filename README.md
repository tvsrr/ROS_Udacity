## USE SMALL_WORLD to verify the home service bot 

![image](https://github.com/user-attachments/assets/ea2e9897-aa4b-4efb-8503-957e04d1d9e5)

To run the home service bot please run the following : 
```bash
./scripts/home_service.sh 
```


# Prerequisites

Before you begin, ensure the following are installed:

- **Git**
- **Git LFS**

---

## Step 1: Install Git LFS

If not already installed, set up Git LFS on your system:

```bash
git lfs install
Note: This is a one-time setup per user account.
```

## Step 2: Clone the Repository

Clone the repository using the Git CLI:

```bash
git clone https://github.com/tvsrr/ROS_Udacity
```

Step 3: Verify Large Files

Git LFS typically downloads large files automatically during checkout. After cloning:

Check file sizes: If files are small (~130 bytes), they are LFS pointers and not the actual content.
Manually fetch LFS files (if needed):

```bash

cd repository  # Navigate into the cloned directory
git lfs fetch --all  # Download all LFS objects
git lfs pull         # Replace pointers with actual files
```

### ------------------------------------------------------------------------------------------------
## PGM Map creator issues 
### ------------------------------------------------------------------------------------------------
Incase the system can't find vector2d.proto. Copy the file into msgs folder of PGM Map creator. 
Modify the msgs cmakelists to include the new vector2d.proto. This will enable creation of maps
to the new world.

### ------------------------------------------------------------------------------------------------
# Packages explanation 
### ------------------------------------------------------------------------------------------------
## Montecarlo Localization Algorithm

In Montecarlo localization each particle is a pose hypothesis. We propose the robot probable location using the motion model. This prediction step could be affected by noise or modeling errors.

In the context of particle filter algorithm, based on the initial hypothetical position of each particle filter robot the new position is evaluated using the current control and the motion model.

Correction step for each particle is generating weight proportional to likelihood of obtaining the robot location from the sensor given we know the particle location and the map is known. A particle has a higher weight if it believes it is at the right position to be observed by the sensor given the current pose and the map.

![](https://miro.medium.com/v2/resize:fit:875/1*JpIrLr-n-KNPSVk8wxu4pQ.png)

In Monte Carlo Localization, a common choice is to use the robot’s motion model as the proposal distribution and then compute importance weights based on sensor readings and the predicted robot pose. While the overall approach still follows the general particle filter framework, these specific choices (motion model for sampling and sensor model for weighting) make MCL well-suited for robot localization.

In conclusion, Monte Carlo Localization continuously updates a belief about the robot’s position using a particle filter. The algorithm determines the most likely position and orientation by sampling from a proposed distribution, allocating important weights, and normalizing them. Because of this, MCL is resistant to motion-related uncertainty and sensor noise.
![](https://miro.medium.com/v2/resize:fit:2400/1*b2nNy5Y0MWMEf2lQXzj9Fg.gif)
## Mapping 
For mapping I initially used the slam-gmapping. Although the mapping is excellent the memory requirments on my virtual machine didn't allow me to store the 400mb map file. I have used PGM Map Creator to create a 2d map of small world. The pgm_map_creator package is designed to convert ROS occupancy grid data into a PGM (Portable GrayMap) image file format. This package plays a crucial role in visualizing and evaluating the generated maps during SLAM and navigation tasks. By transforming map data into a widely supported image format, it allows for easy debugging, offline analysis, and sharing of mapping results. The tool typically applies thresholding and scaling to the raw occupancy data, producing clear visual representations of the environment that can be used for further processing or documentation.

Although I have faced environmental challenges to setup vector2d.proto file and other proto files. I have eventually figured out that adjusting right paths to directories in cmakelists.txt is crucial for getting the build right. 
![small world created by pgm map creator](https://github.com/tvsrr/ROS_Udacity/blob/212d04ac7c8b8abbd00d7e9946c90cdc5b74af40/src/my_robot/maps/small_world.png)
## Transformation
In ROS, the functionality to handle coordinate transforms is provided by packages like **tf** and **tf2**. These packages are essential for converting data between different coordinate frames (for example, converting odometry data in the "odom" frame to the "map" frame). They enable the system to maintain a consistent understanding of the robot's position and orientation relative to various frames in real time, which is crucial for tasks like localization, mapping, and navigation. This has been particularly helpful in managing the marker and robot odometry transformation to ensure that goal positions defined on map are evaluated properly by robot reaching those positions. 
## Custom Packages
In some simulations during mapping through rtabmap, I have faced issues where mapping is not done properly. I had to create a package called helper_codes where I have added some custom code to unpause gazebo's physics engine that enabled better mapping. 

## Costmap 2d
The costmap2d package used in creating occupancy grids from sensor data represents obstacles and freespace by assigning different cost values. The inflation techniques to expand perceived size and ensuring safe distance from robot has been particularly useful in checking in tight spaces why robot is able/unable to generate the path. 


