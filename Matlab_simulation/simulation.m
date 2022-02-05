clc;clear all; close all;
FDH = @(t2,t3,t4,t5,t6) ...
        [0.1 90 0.615 -38; 
        0.705 0 0.0 t2; 
        0.135 -90 0 t3;
        0 90 0.755 t4; 
        0 -90 0 t5; 
        0 0 0.085 t6];
%DHM = [0.1 pi/2 0.615 -38*(pi/180); 0.705 0 0.0 (120)*(pi/180); 0.135 -pi/2 0 0;
%0 pi/2 0.755 pi; 0 -pi/2 0 30*(pi/180); 0 0 0.085 0*(pi/180)];
%f= figure;
for i = 0:90
DHM = FDH(i,0,0,0,i);
%DHM
T01 = trans_matrix(DHM,0,1);
T02 = trans_matrix(DHM,0,2);
T03 = trans_matrix(DHM,0,3);
T04 = trans_matrix(DHM,0,4);
T05 = trans_matrix(DHM,0,5);
T06 = trans_matrix(DHM,0,6);
Tr = [T01 T02 T03 T04 T05 T06];
%T05
Q1=[0 Tr(1,4) Tr(1,8) Tr(1,12) Tr(1,16) Tr(1,20) Tr(1,24)];
Q2=[0 Tr(2,4) Tr(2,8) Tr(2,12) Tr(2,16) Tr(2,20) Tr(2,24) ];
Q3=[0 Tr(3,4) Tr(3,8) Tr(3,12) Tr(3,16) Tr(3,20) Tr(3,24)];
Q=[Q1;Q2;Q3];
% b = uicontrol('Parent',f,'Style','slider','Position',[81,54,419,23],...
%               'value',t5, 'min',0, 'max',1);
% bgcolor = f.Color;
plot3(Q(1,:),Q(2,:),Q(3,:),'-','Color',[255 , 153, 51]./255,'LineWidth',10,'MarkerSize',4,'MarkerFaceColor',[0,0,1]);grid on;
hold on;
plot3(Q(1,:),Q(2,:),Q(3,:),'o','MarkerSize',10,'MarkerFaceColor',[0 0 0]);
axis([-1 2 -1 2 -0.2 2]);
xlabel("X");
ylabel("Y");
zlabel("Z");
pause(0.08);
clf;
Tr,Q
end

