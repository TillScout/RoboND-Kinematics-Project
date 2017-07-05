## Project: Kinematics Pick & Place

---


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[armpic]: ./misc_images/arm_drawing.jpg
[angles]: ./misc_images/angles.jpg
[triangle]: ./misc_images/triangle.jpg


### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I started by drawing the joints and link of the arm by pen and paper. I followed the video instructions to add all X and Z axes. I had some confusion regarding the `X_3` axis. There are various contradictions in the lessons, regarding how `X_i` is defined. In some cases it is the common normal between `Z_{i-1}` and `Z_i`, sometimes it is the common normal between `Z_i` and `Z_{i+1}`. In the video walkthrough for the kuka arm it was the latter definition, so I decided to stick with that.

However in my opinion the orientation of `X_3` should be reversed such that it points from `Z_3` to `Z_4` and not the other way around. However I decided to stick with the version of the video.

Here is a picture of my drawing, where I also indicated the `a` and `d` parameters:

![alt text][armpic]

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

Individual transformation matrices about each joint are defined in the [FK_analysis.py file](kuka_arm/scripts/FK_analysis.py) file

The code to generate the homogeneous transform between base_link and gripper_link is found in the [hom_transform_gripper.py file](kuka_arm/scripts/hom_transform_gripper.py). What it does is take the roll, pitch and yaw information from the end-effector pose and multiply the respective rotation matrices in an extrinsic way. Then I took the cartesian location information from the pose and assembled everything into a homogeneous transform.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

First step on this: Given the pose information (position and orientation of the end-effector (EE)), find the location of the wrist-center (WC). This was pretty straightforward along the lines of the instructions: Take the EE-orientation, assemble the respective rotation matrix and then you can find the vector pointing from WC to EE. This one you go backwards as far as the length of the EE. Since a formula says more than a thousand words:

    WC = p_EE - (s[d6] + s[d7]) * R_yaw * R_pitch * R_roll * Matrix([1,0,0])

Here `WC` is wrist-center location, `p_EE` is the (given) EE location, `s[d6] + s[d7]` is taken from the DH-table and gives the distance from WC to EE (actually `d6` is 0) and the R-thingies are the rotation matrices belonging to the roll, pitch, yaw information from gripper pose.

This gives us the location of the WC. Now we have to find angles theta1-theta3. For the first one, we easily find `theta1 = atan2(WC_y, WC_x)`.
This already poses some issues. I wrote a program that randomly chooses angles within the bounds specified in the urdf file and then does forward_kinematics and then backward-kinematis. In rare occasions, the arm was reaching way over its back and then had the WC e.g. in quadrant 3 but joint 2 in quadrant 1. In these cases the calculation of theta1 was off by pi. In the simulation however this never occured, so I did not program any decision routine for this situation.

For theta2 and theta 3 I drew a triangle between joints 2, 3 and 5, joint 5 being the wrist center and fixed by the given pose information. The dashed line does not have a meaning in the robot itself, it is just a virtual line (note also that the line between joints 3 and 5 also does not follow the arm, since the arm is bent at joint 4). Now the angles theta2 and theta3 should somehow correspond to the angles of this triangle at the respective joints.

![alt text][triangle]


Note that this is the "upper" situation of the given situation. There is also the "down" situation, where joint 3 is below the dashed line, with joints 2 and 5 at the same position. This situation also never occurred in the simulations, even though I had it coded initially. I then at a later stage removed it from my tool.

Now to find theta2, I compute two intermediate angles beta and gamma. One, beta, is the angle between the joint2-joint3-line and the joint2-WC line, the other, gamma, is the angle between the X_2 axis and the joint2-WC-line. Since all edges of my triangles can be computed with the given information (DH table and derived lenghts), I can compute both angles with the cosine rule. Now theta2 = gamma - beta. This is not true in cases where joint 3 is left of joint 2 in the picture, but again, this never occured in the simulation, so I did not code it up.

For theta3 I compute what I call the "naive" theta3 angle as the angle of my triangle around joint 3. As the arm is not following the edge of the triangle, this value is not equal to theta3, but it is off by a value, which is independent of the actual pose of the arm. So with the forward_kinematics tool I compared the naive theta3 angle with the real one and found that they are off by around 1.53 and could then hardcode this angle. There is also a way to compute it, but I found this way much more hassle-free.

![alt text][angles]


With thetas 1-3 found, all that remains is theta4-6, i.e. the pose information. This was a bit more tricky. To find the angles, I first looked for the matrix that does the transformation from joint3 to the EE. This matrix contains only the variables for angles 4-6. To find this matrix I took the rotation matrices from the transformation from joint 0 to 3 as described above in section 2 and took the composite `R0_3`. When `R_EE` is the rotation matrix between base frame and end-effector as computed in section 2 with just end-effector pose information, we find

    R3_6 = R0_3**(-1) * R_EE
Since theta1-3 and R_EE are known, this computation gives a variable-free representation of `R3_6`

On the other hand the matrix R3_6 can be computed by multiplying the respective individual rotation matrices. The latter approach still contains variables for theta4-6, hence by comparing the two approaches we can derive equations for these angles. In particular if we look at position `(2,3)` of this matrix, we find the entry `cos(theta5)`. This allows us to compute theta5.

There are two things to keep in mind though: If `cos(theta5)=1`, then theta5 is zero and joints 4 and 6 are collinear. Hence we cannot distinguish between theta4 and 6. Also plus and minus theta5 give the same cos, hence we always have to look at a choice here.

In case of theta5=0, I looked at the `(1,1)` and `(3,1)` positions of R3_G and set `cos(theta5)=1`. Then with some basic trigonometry rules on products of sin and cos we find `sin(theta4+theta6) = R3_G(3,1)`, where the right hand side is the matrix computed above from the `R_EE`-matrix. I decided to set theta6=0 in this case (since both angles are collinear) and hence with taking the arcsin on both sides, we get a solution for the case theta5=0. In this special case this means that we have a full set of angles and solved the IK problem.

If theta5 is not zero, we can go on with computing theta4 and theta6. I took the upper right entry  of R3_G, which reads `-sin(theta5)*cos(theta4)`. Also if we look at the `(2,2)`-entry we get `-sin(theta5)*sin(theta6)` We can solve those for theta4 and 6 respectively and get

    cos(theta4) = -R3_G(1,3)/sin(theta5)
    sin(theta6) = -R3_G(2,2)/sin(theta5)
With arccos and arcsin, we can now solve for theta4 and theta6. I read on Slack that some others found formulas using atan2, but the ones that I found did not work for me.

Now that we found these possibilities for theta4 and 6, we are faced with some choices. As explained above, theta5 could come with a negative sign due to the symmetry of cos. Entering -theta5 in the formulas above will in fact supply us with a range of possibilities for theta4 and 6 due to some trigonometry identities. For example we find that when choosing -theta5, one choice could be theta6-pi/2.

I coded those choices in my IKserver.py and then iterate over them and check which combination satisfies the required pose information. This is not very elegant, but I did not find another way for this.

This completes the inverse Kinematics problem.


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
