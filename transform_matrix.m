% This program is to compute the transfer matrix of IRB 2400 in
% mathematical models
clc;
clear;
% Initialize parameter of IRB 2400
d1 = 435/1000;
d4 = 755/1000;
d6 = 85/1000;
a1 = 205/1000;
a2 = 705/1000;
a3 = 135/1000;

% Set up value for joint variables, here we assume the angle of every joint
% is pi/3
q1 = pi/3;
q2 = -pi/4;
q3 = pi/4;
q4 = pi/4;
q5 = pi/4;
q6 = pi/4;

% Initialize homogeneous matrix
A1 = [cos(q1) 0 -sin(q1) a1*cos(q1); sin(q1) 0 cos(q1) a1*sin(q1); 0 -1 0 0;0 0 0 1];
A2 = [cos(q2) -sin(q2) 0 a2*cos(q2);sin(q2) cos(q2) 0 a2*sin(q2); 0 0 0 0; 0 0 0 1];
A3 = [sin(q3) 0 cos(q3) a3*cos(q3); -cos(q3) 0 sin(q3) a3*sin(q3); 0 -1 0 0;0 0 0 1];
A4 = [cos(q4) 0 sin(q4) 0;sin(q4) 0 -cos(q4) 0;0 -1 0 d4; 0 0 0 1];
A5 = [cos(q5) 0 -sin(q5) 0; sin(q5) 0 cos(q5) 0; 0 -1 0 0; 0 0 0 1];
A6 = [cos(q6) -sin(q6) 0 0; sin(q6) cos(q6) 0 0;0 0 1 d6; 0 0 0 1];

% The transform matrix of end effector 
T6 = A1*A2*A3*A4*A5*A6

% the translation vector o
o = T6(1:3,4);

% rotation matrix
rot_m = T6(1:3,1:3);

% convert rotation matrix to angle
m2angle = vrrotmat2vec(rot_m)

compute_axis = m2angle(1:3);compute_angle = m2angle(4)

Rotation_matrix = rot_m
 


