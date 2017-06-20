## Project: Kinematics Pick & Place

---


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png


### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I started by drawing the joints and link of the arm by pen and paper. I followed the video instructions to add all X and Z axes. I had some confusion regarding the `X_3` axis. There are various contradictions in the lessons, regarding how `X_i` is defined. In some cases it is the common normal between `Z_{i-1}` and `Z_i`, sometimes it is the common normal between `Z_i` and `Z_{i+1}`. In the video walkthrough for the kuka arm it was the latter definition, so I decided to stick with that.

However in my opinion the orientation of `X_3` should be reversed such that it points from `Z_3` to `Z_4` and not the other way around. However I decided to stick with the version of the video.

Here is a picture of my drawing, where I also indicated the `a` and `d` parameters:
# pic to follow

The alphas were already given in the video walkthrough. For the `a`s, that measure the distances between the `Z`-axes, one can easily that `a_0`, `a_4`, `a_5` and what I call `a_G` (for the gripper joint) are 0, because the respective `Z`-axes are parallel or collinear.
The same applies to some of the `d`s, that measure the distances between the `X`-axes. Here `d_2`, `d_3`, `d_5` and `d_6` are all zero. So there are only a few non-zero values remaining, for which we have to look up their measurements in the urdf file. This is the table of measurements taken from the urdf file, giving the origin for each joint relative to the parent link:

joint |  x | y | z
---|---|---|---
1|0|0|0.33
2|0.35|0|0.42
3|0|0|1.25
4|0.96|0|-0.054
5|0.54|0|0
6|0.193|0|0
G|0.11|0|0

Since this is measured from the parent origin and the origin of the child sits at the centre of the joint, this should allow us to derive the `a` and `d` values that we are looking for. The x and z axes of the urdf file however do not match the `X` and `Z` axes of the DH analysis. To compute the missing values:

`a_1` is the distance between `Z_1` and `Z_2` (measured in `X_1` direction). `X_1` is pointing in x-direction in the urdf file, so the value for `a_1` is the difference in x direction between links 1 and 2, this would be 0.35.
`a_2` is the distance between `Z_2` and `Z_3` along `X_2`. According to the drawing, `X_2` points upwards, which is the z-direction in the urdf file. Hence `a_2` is the height difference between joints 2 and 3, so `a_2` is 1.25.
By similar reasoning, `a_3` is the height difference between joints 3 and 4, so `a_3 = -0.054`.

Looking at `d_1` in the picture, we see that it is the height of the centre of joint 2. To get its value we have to add the relative z-coordinates of joints 1 and 2 and end up with `d_1 = 0.33 + 0.42 = 0.75`.
`d_4` measures the offset between joints 3 and 5, so its value is `d_4 = 0.96 + 0.54 = 1.5`.
`d_G` is the distance between the gripper and joint 5, so we get `d_G = 0.193 + 0.11 = 0.303`.

What remains are the `theta`s, that measure the angles between the `X_i`-axes. Since all of those but the `X_1`-axis are parallel, we can take the variable `q` for most of those. Because of the 90° offset of joint 2, we get `theta_2 = q_2 - 90°`, where the `q_i` are the variable angles of the joints.

Since the gripper is fixed, we get a 0 for `theta_G`

This is the completed DH-table:

i | `alpha_{i-1}`|`a_{i-1}`| `d_i`| `theta_i`
---|---|---|---|---
1|0|0|0.75|`d_1`
2|-90°|0.35|0|`q_2`-90°
3|0|1.25|0|`q_3`
4|-90°|-0.054|1.5|`q_4`
5|90°|0|0|`q_5`
6|-90°|0|0|`q_6`
G|0|0|0.303| 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Individual transformation matrices about each joint are currently defined in the [FK_analysis.py file](kuka_arm/scriptts/FK_analysis.py) file
#### TODO: move it here, add explanation

The code to generate the homogeneous transform between base_link and gripper_link is found in the [hom_transform_gripper.py file](kuka_arm/scripts/hom_transform_gripper.py). What it does is take the roll, pitch and yaw information from the end-effector pose and multiply the respective rotation matrices in an extrinsic way. Then I took the cartesian location information from the pose and assembled everything into a homogeneous transform.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's another image!

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
