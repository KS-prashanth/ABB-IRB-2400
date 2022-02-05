function A=dh(theta, d, a, alpha)
switch nargin
    case 3 %abbreviated call to denavit
        %arrange arguments
        robot = theta;
        q = d;
        i = a;
         
        theta = eval(robot.DH.theta);
        d = eval(robot.DH.d);
        a = eval(robot.DH.a);
        alpha = eval(robot.DH.alpha);
         
        theta = theta(i);
        d = d(i);
        a = a(i);
        alpha = alpha(i);
        
        A=[cos(theta)  -cos(alpha)*sin(theta)   sin(alpha)*sin(theta)   a*cos(theta);
            sin(theta)   cos(alpha)*cos(theta)  -sin(alpha)*cos(theta)   a*sin(theta);
            0              sin(alpha)             cos(alpha)             d;
            0                     0                     0              1];
    case 4 %full 4 argumen call to denavit
        A=[cos(theta)  -cos(alpha)*sin(theta)   sin(alpha)*sin(theta)   a*cos(theta);
            sin(theta)   cos(alpha)*cos(theta)  -sin(alpha)*cos(theta)   a*sin(theta);
            0              sin(alpha)             cos(alpha)             d;
            0         0                     0              1];
    otherwise
        disp('ERROR:denavit: uncorrect number of arguments')
end