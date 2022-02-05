% This program is to compute the jacobain matrix of end effector
clc;
clear;
% Initialize parameter of IRB 2400
d1 = 435/1000;
d4 = 755/1000;
d6 = 85/1000;
a1 = 205/1000;
a2 = 705/1000;
a3 = 135/1000;

% Set up value for joint variables q_dot, here we assume the angle of every joint
% is as following
q1_dot = 0.5;       %rad/s
q2_dot = -0.3;       %rad/s
q3_dot = -0.2;       %rad/s
q4_dot = 0.2;       %rad/s
q5_dot = 0.1;       %rad/s
q6_dot = 0.1;       %rad/s

q_dot = [q1_dot; q2_dot; q3_dot; q4_dot; q5_dot; q6_dot];

% The vector of z
z0 = [0;0;1];
z1 = [1;0;0];
z2 = [1;0;0];
z3 = [0;0;1];
z4 = [0;0;1];
z5 = [0;0;1];

% Initialize the velocity 
velo = [0;0;0;0;0;0];

% set up time t
for t = 0:0.1:5

    % Set up value for joint varibale q
    q1 = q1_dot*t;      % rad
    q2 = q2_dot*t;      % rad
    q3 = q3_dot*t;      % rad
    q4 = q4_dot*t;      % rad
    q5 = q5_dot*t;      % rad
    q6 = q6_dot*t;      % rad



    % Initialize homogeneous matrix
    A1 = [cos(q1) 0 -sin(q1) a1*cos(q1); sin(q1) 0 cos(q1) a1*sin(q1); 0 -1 0 0;0 0 0 1];
    A2 = [cos(q2) -sin(q2) 0 a2*cos(q2);sin(q2) cos(q2) 0 a2*sin(q2); 0 0 0 0; 0 0 0 1];
    A3 = [sin(q3) 0 cos(q3) a3*cos(q3); -cos(q3) 0 sin(q3) a3*sin(q3); 0 -1 0 0;0 0 0 1];
    A4 = [cos(q4) 0 sin(q4) 0;sin(q4) 0 -cos(q4) 0;0 -1 0 d4; 0 0 0 1];
    A5 = [cos(q5) 0 -sin(q5) 0; sin(q5) 0 cos(q5) 0; 0 -1 0 0; 0 0 0 1];
    A6 = [cos(q6) -sin(q6) 0 0; sin(q6) cos(q6) 0 0;0 0 1 d6; 0 0 0 1];

    % compute homogenous matrix
    T1 = A1;
    T2 = A1*A2;
    T3 = A1*A2*A3;
    T4 = A1*A2*A3*A4;
    T5 = A1*A2*A3*A4*A5;
    T6 = A1*A2*A3*A4*A5*A6;

    % Extract o from homogenous Matrix
    o_0 = [0;0;0];
    o_1 = T1(1:3,4);
    o_2 = T2(1:3,4);
    o_3 = T3(1:3,4);
    o_4 = T4(1:3,4);
    o_5 = T5(1:3,4);
    o_6 = T6(1:3,4);

    % Compute Jacobain Matrix
    Jv1 = cross(z0,(o_6-o_0));
    Jv2 = cross(z1,(o_6-o_1));
    Jv3 = cross(z2,(o_6-o_2));
    Jv4 = cross(z3,(o_6-o_3));
    Jv5 = cross(z4,(o_6-o_4));
    Jv6 = cross(z5,(o_6-o_5));

    Jv = [Jv1 Jv2 Jv3 Jv4 Jv5 Jv6];

    Jw = [z0 z1 z2 z3 z4 z5];

    J = [Jv;Jw]
    
    v = J*q_dot;
    
    velo = [velo v]        % the velocity of end effector
    
end


    
    




