## Project: Kinematics Pick & Place

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Typically for any transformation from link(i-1) to link(i), we need six independent parameters. However using modified DH parameter we can do this by using following 4 variables.


alpha(i-1): Twist Angle: Angle between Zi-1 & Zi measured along Xi-1 in right-hand sense

a(i-1) : Link Length: Distance between Zi-1 & Zi along Xi-1

d(i) : Link Offset: signed distance from Xi-1 & Xi along Zi

theta(i) : Joint Angle : angle between Xi-1 & Xi measured along Zi in right-hand sense

For Kuka KR210, link assignment and joint angles can be seen is image below

[image_1]: DH_Links.png
![alt text][image_1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

From the above link assignment, we can construct modified DH parameter table as shown below


Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1=0.75 | q1
1->2 | - pi/2 | a1=0.35 | 0 | -pi/2 + q2
2->3 | 0 | a2=1.25 | 0 | q3
3->4 |  -pi/2 | a3=-0.054 | d4=1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | d7=0.303 | 0

Figure below shows transformation matrix between adjacent links, individual TF and finally from Base to End Effector

[image_2]: Link_Transformation.png
![alt text][image_2]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
Inverse Kinematics is more complicated in comparison to Forward Kinematics. Here the pose(i.e position and orientation) of the end effector is known and our goal is to get joint angles for the manipulator. In case of Kuka KR210, we have last 3 revolute joints which intersect at single point called wrist center. Therefore for Kuka210 we can decouple inverse kinematics into 2 parts: Inverse Position i.e calculation of theta1, theta2 & theta3 and Inverse Orientation i.e calculation of theta4, theta5 & theta6

In Inverse Position, first we need to get wrist center position. For that we can represent our homogeneous transform as shown below where l,m,n are orthonormal vectors representing the end-effector orientation along X, Y, Z axes of the local coordinate frame.

[image_3]: lmn.png
![alt text][image_3]

As n is along gripper z axis we can get wrist position as shown below

[image_4]: wrist.png
![alt text][image_4]

Now we want to get first three joint angles which will place the manipulator in wrist center position. First joining angle theta1 is straight forward and can be obtained easily by projecting on XY plane and is given as

theta1 = atan2(Wy,Wx)

For theta2 and theta3, we need help from trigonometry specifically and its representation is seen below in image

[image_5]: inverse_position.png
![alt text][image_5]

From above image using SSS triangle rule we can get theta2 and theta3 as shown below

[image_6]: theta123.png
![alt text][image_6]

For Inverse orientation, we use Euler angles method where resulting rotational matrix is in form
[image_7]: Rot_matrix.png
![alt text][image_7]

Also, one important thing to note here is we need correction matrix since reference frame for end gripper link is not consistent with modified DH convention shown above. We can get roll, pitch and yaw angles euler_from_quaternions() method. These values are then substituted in our ROT_EE matrix which is obtained as shown below

[image_8]: rot_corr.png
![alt text][image_8]

From comparing R3_6 and overall matrix with last 3 joint angles shown above, we can get values for theta4, theta5 and theta6 as shown below

[image_8]: theta456.png
![alt text][image_8]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

In terms of implementation, following were the observations
1. Implementing forward kinematics and general rotation matrices outside main loop helps to improve overall speed performance.
2. In calculation of R3_6, better results were seen using transpose instead of suggested inv("LU")
3. Before releasing object in bin, its important to ensure that we have last 3 joints in proper orientation or else object may get dropped down outside bin. I added a special check for this to ensure drop box release part.
4. In terms performance, I was able to pick and place 9/10 objects. In one case it object was dropped before placing in the bin.
5. Using clip limits from KR210 urdf is extremely helpful and results in smoother inverse kinematics motion.
6. Also for slow systems, better to use step by step option using 'Next' rather than 'Continue' option.

Here are the screen shots of final output from the run including final image after all 10 trials

[image_9]: output1.png
![alt text][image_9]

[image_10]: output2.png
![alt text][image_10]

