

k=1;
for i=1:size(joint_pos,1)
if(mod(i-1,10)==0)
jp(k,1:5)=joint_pos(i,:);
k=k+1;
end
end


h = figure(1);
for i = 1:size(jp,1)
    plot_youbot_arm(h,youbot_joints_2_matlab( jp(i,:)));
end
grid on;
axis equal
legend('x axis','y axis','z axis')
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')