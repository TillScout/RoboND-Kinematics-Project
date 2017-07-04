#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import atan2, sqrt, acos, asin
from sympy import symbols, cos, sin, simplify
from sympy.matrices import Matrix
import numpy as np
pi = np.pi

### Create symbols for joint variables
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # thetas
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# joint limits for checks:
deg = 0.017453293
j1_lims = [-185*deg, 185*deg]
j2_lims = [-45*deg, 85*deg]
j3_lims = [-210*deg,(155-90)*deg]
j4_lims = [-350*deg, 350*deg]
j5_lims = [-125*deg, 125*deg]
j6_lims = [-350*deg, 350*deg]

# DH Parameters as explained in the writeup
s = {alpha0: 0,     a0:     0,  d1: 0.75, 
     alpha1: -pi/2, a1:  0.35,  d2: 0,       q2: q2-pi/2,
     alpha2: 0,     a2: 1.25,   d3: 0,
     alpha3: -pi/2, a3:  -0.054, d4: 1.5,  
     alpha4: pi/2,  a4: 0,      d5: 0,
     alpha5: -pi/2, a5: 0,      d6: 0,
     alpha6: 0,     a6: 0,      d7: 0.303, q7: 0}

#### Homogeneous Transforms around each individual joint
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
T5_6 = T5_6.subs(s)

T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
T6_G = T6_G.subs(s)

# to solve orientation problem:
T0_3 = simplify(T0_1 * T1_2 * T2_3)
R0_3 = T0_3[0:3, 0:3]
R0_3_inv = simplify(R0_3.inv())
R3_G = simplify(T3_4 * T4_5 * T5_6 * T6_G)[0:3, 0:3]

# all of these return DH coordinate frames. Including T0_G: base/world z-axis is transformed to point in gripper direction!
# correction matrix to transform gripper's coordinate frame into DH one.
# correction matrix to transform gripper's coordinate frame into DH one.
T_corry = Matrix([[ cos(-np.pi/2),           0,  sin(-np.pi/2),    0],
                  [             0,           1,              0,    0],
                  [-sin(-np.pi/2),           0,  cos(-np.pi/2),    0],
                  [0,           0,           0,                    1]])

T_corrz = Matrix([[    cos(np.pi), -sin(np.pi),              0,    0],
                  [    sin(np.pi),  cos(np.pi),              0,    0],
                  [             0,           0,              1,    0],
                  [             0,           0,              0,    1]])
    
T_corr = simplify(T_corrz * T_corry)
R_corr = T_corr[0:3,0:3]



# some values that are needed for theta-calculation:
j3_wc = float(sqrt(s[a3]**2 + s[d4]**2) )
j3_j4 = float(sqrt(0.96**2 + 0.054**2))
j4_wc = 0.54

# parameters for theta3:
    
# the following is the angle, by which the "naive theta3 approach" is off. 
# Then you can modify the naive theta3 with this
# number and get the theta3 prediction also in the other cases.
# the off-angles are found by just taking an arbitrary example and the computing
# the error
theta3_off_angle_up = 1.5348118667128452 
theta3_off_angle_down = -4.7483734404667421

# function to get thetas 2&3 from theta1:
def t23from1(WC, th1):
    
    [wc_x, wc_y, wc_z] = WC
    # calculate the position of joint 2:
    j2_x, j2_y, j2_z, _ = T0_1.evalf(subs={q1:th1}) * T1_2.col(-1)

    # distance between joint2 and WC
    wc_j2 = (Matrix([j2_x, j2_y, j2_z]) - Matrix([wc_x, wc_y, wc_z])).norm()

    ### theta 2: ###

    # intermediate values:
    cosbeta = (s[a2]**2 + wc_j2**2 - j3_wc**2)/(2 * s[a2] * wc_j2)

    # the following is the angle between the straight line between joint2 and WC and the possible direction of link2, also the negative is a solution!
    beta = float(acos(cosbeta))

    # compute angle between joint2-WC line to Z_5-Y_5-plane (DH coordinate frame):
    h = wc_z - j2_z # height diff between joint 2 and WC
    wc_j2_z = (Matrix([j2_x, j2_y, wc_z]) - Matrix([wc_x, wc_y, wc_z])).norm() # distance between joint2 and j2 projection onto wc height
    cosgamma = (h**2 + wc_j2**2 - wc_j2_z**2)/(2 * h * wc_j2)
    gamma = float(acos(cosgamma))
    
    theta2_predict_up = float(gamma - beta)
    theta2_predict_down = float(gamma + beta)
    # wrong if theta1 has the "overarching issue". Also wrong, if theta1 is right, 
    # but the wrist center is just a little above joint 2, i.e. between j2 and the 
    # base, in that case it is the negative. Probably also not an issue in the 
    # simulation, but might correct it later.

    ### theta3: ###
   
    # compute "naive" theta3 as angle between j2-j3-wc
    cos_naive_theta3 = (s[a2]**2 + j3_wc**2 - wc_j2**2)/(2 * s[a2] * j3_wc)
    naive_theta3 = acos(cos_naive_theta3)
    
    theta3_predict_up = float(theta3_off_angle_up - naive_theta3)
    theta3_predict_down = float(theta3_off_angle_down + naive_theta3)
    return [theta2_predict_up, theta3_predict_up], [theta2_predict_down, theta3_predict_down]

# function to get thetas 4-6 from 1-3, returns lists of possible theta4-6 choices:
def th456from123(R_Mat, th1, th2, th3):
    R3_G = R0_3_inv.evalf(subs={q1: th1, q2:th2, q3: th3}) * R_Mat
    theta5_predict = float(acos(R3_G[1,2])) # also the negative is a possibility!!!
    if theta5_predict < 0.01: # might sometimes get errors in floating point operations when dividing small numbers later
        # theta5 = 0 => 4 and 6 collinear, so only theta4+theta6 matters
        theta4_predict = - float(asin(R3_G[2,0]))
        theta6_predict = 0
        return [theta4_predict, theta5_predict, theta6_predict]
    else:
        theta4_predict = float(acos(-R3_G[0,2]/sin(theta5_predict))) # if negative theta5 is chosen, this also gets a sign
        theta6_predict = float(asin(-R3_G[1,1]/sin(theta5_predict))) # if negative tcheta5 is chosen, this also gets a sign
        # options in case of negative theta5:
        theta4_predict_neg = [pi-theta4_predict, float(asin(-R3_G[2,2]/sin(theta5_predict)))]
        theta6_predict_neg = [pi-theta6_predict, theta6_predict-pi/2]
    return theta5_predict, theta4_predict, theta4_predict_neg, theta6_predict, theta6_predict_neg


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            print "calculating step ",x, "of ", len(req.poses)
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            print "position: ", round(px,6),",", round(py, 6),",", round(pz,6)
            print "rpy: ", round(roll, 6),",", round(pitch, 6),",", round(yaw,6)
    
            # to compute roll, pitch and yaw from a rotation matrix
            def rpy_from_matrix(matrix):
                r11, r21, r31, r32, r33 = matrix[0,0], matrix[1,0], matrix[2,0], matrix[2,1], matrix[2,2]
                yy = atan2(r21,r11)# * 180./pi
                pp  = atan2(-r31, sqrt(r11**2 + r21**2))# * 180./pi
                rr = atan2(r32, r33) #* 180./pi
                return rr, pp, yy
            
            # to check wether given angles produce the desired roll pitch and yaw, required for handling theta-choices
            def rpy_check(th, r=roll,p=pitch,y=yaw):
                tol = 0.01
                theta_subs = {q1: th[0], q2: th[1], q3: th[2], q4: th[3], q5: th[4], q6: th[5]}
                T0_G = simplify(T0_1.subs(theta_subs) * T1_2.subs(theta_subs) * T2_3.subs(theta_subs) * T3_4.subs(theta_subs) * T4_5.subs(theta_subs) * T5_6.subs(theta_subs) * T6_G.subs(theta_subs))*T_corr
                rr, pp, yy = rpy_from_matrix(T0_G)
                ra, pa, ya = abs(r-rr), abs(p-pp), abs(y-yy)
                return (ra < tol or abs(ra-pi) < tol)*(pa < tol or abs(pa-pi)<tol)*(ya < tol or abs(ya-pi)<tol)

    
            R_roll = Matrix([[ 1,              0,        0],
                      [ 0,        cos(roll), -sin(roll)],
                      [ 0,        sin(roll),  cos(roll)]])

            R_pitch = Matrix([[ cos(pitch),        0,  sin(pitch)],
                      [       0,        1,        0],
                      [-sin(pitch),        0,  cos(pitch)]])

            R_yaw = Matrix([[ cos(yaw), -sin(yaw),        0],
                      [ sin(yaw),  cos(yaw),        0],
                      [ 0,              0,        1]])

            # position and rotation matrix for EE in urdf coordinate frame
            p_EE = Matrix([px,py,pz])
            R_EE = R_roll*R_pitch*R_yaw
            
                        # calculate wrist center:
            WC = p_EE - (s[d6] + s[d7])*R_yaw*R_pitch*R_roll*Matrix([1,0,0])
            [wc_x, wc_y, wc_z] = WC
            
            ################ Calculate joint angles using Geometric IK method:
            
            ### theta1: ###
            
            theta1_predict = float(atan2(wc_y, wc_x))
            #this might be off by pi in case the arm is arching over its 
            # back - probably not happening in the simulation, but might happen

            ### theta2 and theta3: ###
            
            up, down = t23from1(WC, theta1_predict)
            # "up" and "down" represent the two possibilities which you
            # might get for thetas 2 and 3
            
            # now we have two sets of angles computed: theta1 and two pairs of 
            # theta2 & theta3, one of them might be out of bounds:
            if not j2_lims[0] < up[0] < j2_lims[1] or not j3_lims[0] < up[1] < j3_lims[1]:
                up = None
            if not j2_lims[0] < down[0] < j2_lims[1] or not j3_lims[0] < down[1] < j3_lims[1]:
                down = None
            
            ### theta4-6: ###
            # this only works for correctly predicted theta1!
            # the theta4&6 computations are sometimes off by a factor of 2pi,
            # this is due to the wide range of possible angles for
            # those joints, should not be a problem.
            
            # tie up all the choices and check whether they are within limits:

            returned_angles = th456from123(R_EE*R_corr, theta1_predict, up[0], up[1])
            if len(returned_angles) == 3:
                angles = [theta1_predict]+up+returned_angles
            else:
                [theta5_predict, theta4_predict, theta4_predict_neg, theta6_predict, theta6_predict_neg] = returned_angles
                up = [theta1_predict]+up
                if rpy_check(up+[theta4_predict, theta5_predict, theta6_predict]):
                    angles = up + [theta4_predict, theta5_predict, theta6_predict]
                else:
                    for th4 in theta4_predict_neg:
                        for th6 in theta6_predict_neg:
                            if rpy_check(up + [th4, -theta5_predict, th6]):
                                angles = up + [th4, -theta5_predict, th6]
                                break
            
            if not "angles" in locals():
                returned_angles = th456from123(R_EE*R_corr, theta1_predict, down[0], down[1])
                if len(returned_angles) == 3:
                    angles = [theta1_predict]+down+returned_angles
                else:
                    theta5_predict, theta4_predict, theta4_predict_neg, theta6_predict, theta6_predict_neg = returned_angles
                    down = [theta1_predict]+down
                    if rpy_check(down+[theta4_predict, theta5_predict, theta6_predict]):
                        angles = down + [theta4_predict, theta5_predict, theta6_predict]
                    else:
                        for th4 in theta4_predict_neg:
                            for th6 in theta6_predict_neg:
                                if rpy_check(down + [th4, -theta5_predict, th6]):
                                    angles = down + [th4, -theta5_predict, th6]
                                    break

            if not "angles" in locals():
                print "no angles found, error remaining"
                
            print "angles", angles
                
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            
            joint_trajectory_point.positions = angles
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
