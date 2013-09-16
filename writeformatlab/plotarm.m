function [ ] = plot_youbot_arm(h, theta)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

parameter = load_parameter();
l2 = parameter.l2;
l3 = parameter.l3;
l4 = parameter.l4;
lox = parameter.lox;
loz = parameter.loz;
lax = parameter.lax;

% R1 = rotz(theta(1));
% R2 = roty(-pi/2)*rotx(theta(1))*roty(theta(2));
% R3 = roty(-pi/2)*rotx(theta(1))*roty(theta(2)+theta(3));
% R4 = roty(-pi/2)*rotx(theta(1))*roty(theta(2)+theta(3)+theta(4));
R5 = roty(-pi/2)*rotx(theta(1))*roty(theta(2)+theta(3)+theta(4))*rotx(theta(5));

p1 = [lax 0 0]';
p2 = [lox*cos(theta(1))+lax lox*sin(theta(1)) loz]';
p3 = [(l2*sin(theta(2)) + lox)*cos(theta(1)) + lax;
     (l2*sin(theta(2)) + lox)*sin(theta(1));
     l2*cos(theta(2)) + loz];
p4 = [(l2*sin(theta(2)) + l3*sin(theta(2)+theta(3)) + lox)*cos(theta(1)) + lax;
    (l2*sin(theta(2)) + l3*sin(theta(2)+theta(3)) + lox)*sin(theta(1));
    l2*cos(theta(2)) + l3*cos(theta(2)+theta(3)) + loz];
p5 = [(l2*sin(theta(2)) + l3*sin(theta(2)+theta(3)) + l4*sin(theta(2)+theta(3)+theta(4)) + lox)*cos(theta(1)) + lax;
    (l2*sin(theta(2)) + l3*sin(theta(2)+theta(3)) + l4*sin(theta(2)+theta(3)+theta(4)) + lox)*sin(theta(1));
    l2*cos(theta(2)) + l3*cos(theta(2)+theta(3)) + l4*cos(theta(2)+theta(3)+theta(4)) + loz];

joint_points = [p1 p2 p3 p4 p5]';

figure(h);
plot(joint_points(:,2),joint_points(:,3))
hold on
plot(joint_points(:,2),joint_points(:,3),'.r','MarkerSize',10)

end

