function [q] = inversekinematic_irb2400(robot, T)
%initialize q,
%eight possible solutions are generally feasible
q=zeros(6,8);
 
%Evaluate the parameters
theta = eval(robot.DH.theta);
d = eval(robot.DH.d);
a = eval(robot.DH.a);
alpha = eval(robot.DH.alpha);
 
 
%See geometry at the reference for this robot
L1=d(1);
L2=a(2);
L3=d(4);
L6=d(6);
 
A1 = a(1);
 
 
%T= [ nx ox ax Px;
%     ny oy ay Py;
%     nz oz az Pz];
Px=T(1,4);
Py=T(2,4);
Pz=T(3,4);
 
%Compute the position of the wrist, being W the Z component of the end effector's system
W = T(1:3,3);
 
% Pm: wrist position
Pm = [Px Py Pz]' - L6*W; 
 
 
%Evaluate the parameters
d = eval(robot.DH.d);
a = eval(robot.DH.a);
 
%See geometry
L2=a(2);
L3=sqrt(d(4)^2+a(3)^2);
 
%given q1 is known, compute first DH transformation
T01=dh(robot, q, 1);
 
%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];
 
r = sqrt(p1(1)^2 + p1(2)^2);
 
beta = atan2(-p1(2), p1(1));

gamma = (acos((L2^2+r^2-L3^2)/(2*r*L2)));

%return two possible solutions
%elbow up and elbow down
%the order here is important and is coordinated with the function
%solve_for_theta3
q2(1) = pi/2 - beta - gamma; %elbow up
q2(2) = pi/2 - beta + gamma; %elbow down
 
 

%Express Pm in the reference system 1, for convenience
p1 = inv(T01)*[Pm; 1];

r = sqrt(p1(1)^2 + p1(2)^2);
 
eta = (acos((L2^2 + L3^2 - r^2)/(2*L2*L3))); 


%return two possible solutions
%elbow up and elbow down solutions
%the order here is important
q3(1) = -(atan(d(4)/a(3))+eta - pi);
q3(2) = -(pi+atan(d(4)/a(3))-eta);

if abs(Q(3,3)-1)>thresh
    %normal solution
    if wrist==1 %wrist up
        q(4)=atan2(Q(2,3),Q(1,3));
        q(6)=atan2(-Q(3,2),Q(3,1));
        q(5)=acos(Q(3,3))+pi;
        
    else
        q(4)=atan2(Q(2,3),Q(1,3))+pi;
        q(6)=atan2(-Q(3,2),Q(3,1))+pi;
        q(5)=acos(Q(3,3));
        
    end
    
end
