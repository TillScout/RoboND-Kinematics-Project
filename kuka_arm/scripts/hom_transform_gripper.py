#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 17:28:16 2017

@author: till
"""

from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix
import numpy as np

### Create symbols for joint variables
q1, q2, q3 = symbols('q1:4') # remember slices do not include the end value 

# define rotation matrices:
R_x = Matrix([[ 1,              0,        0],
              [ 0,        cos(q1), -sin(q1)],
              [ 0,        sin(q1),  cos(q1)]])

R_y = Matrix([[ cos(q2),        0,  sin(q2)],
              [       0,        1,        0],
              [-sin(q2),        0,  cos(q2)]])

R_z = Matrix([[ cos(q3), -sin(q3),        0],
              [ sin(q3),  cos(q3),        0],
              [ 0,              0,        1]])

# combine into extrinsic rotation 
R = simplify(R_z*R_y*R_x)

# to get from pose informatio (location and orientation):
def transform(xx, yy, zz, r,p,y):
    # location vector:
    loc = Matrix([xx, yy, zz])
    # use the above defined matrix for roll, pitch and yaw rotation
    R_e = R.evalf(subs={q1: r, q2:p, q3:y})
    # turn into fully fledged homogeneous transfor
    T = R_e.row_join(loc)
    T = T.col_join(Matrix([[0,0,0,1]]))
    return T, R_e # return both for debugging purposes
    