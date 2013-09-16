clear direct_step mm_step gr_vel gr_vel_fb tor
close all
parameter = load_parameter();
joint_offsets = parameter.joint_offsets_deg/180*pi;

fid = fopen('grasp_direct_step.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    direct_step(i,1:5)=read(1:5,i);
    direct_step(i,1:5)=youbot_joints_2_matlab(direct_step(i,1:5));
end
t=t';
fid = fopen('grasp_mm_step.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    mm_step(i,1:5)=read(1:5,i);
    mm_step(i,1:5)=youbot_joints_2_matlab(mm_step(i,1:5));
end

fid = fopen('grasp_vel.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    gr_vel(i,1:5)=read(1:5,i);
    gr_vel(i,1:5)=youbot_joints_2_matlab(gr_vel(i,1:5));
end

fid = fopen('grasp_vel_fb.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    gr_vel_fb(i,1:5)=read(1:5,i);
    gr_vel_fb(i,1:5)=youbot_joints_2_matlab(gr_vel_fb(i,1:5));
end

fid = fopen('tor_pos.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    tor(i,1:5)=read(1:5,i);
    tor(i,1)=-tor(i,1);
    tor(i,2)=tor(i,2);
    tor(i,3)=-tor(i,3);
    tor(i,4)=tor(i,4);
    tor(i,5)=-tor(i,5);
end

% fid = fopen('tor_pos.txt','r')
% read = fscanf(fid,'%f',[5,inf])
% fclose(fid)
% [m,n]=size(read)
% for i=1:n
%     tor(i,1:5)=read(1:5,i);
%     tor(i,1:5)=youbot_joints_2_matlab(tor(i,1:5));
% end

% h = figure(1);
% subplot(1,2,2);
% for i = 1:size(mm_step,1)
%     plotarm(h,mm_step(i,:));
% end
% grid on;
% axis equal
% %legend('x axis','y axis','z axis')
% xlabel('y [m]')
% ylabel('z [m]')
% zlabel('x [m]')
% h = figure(1);
% subplot(1,2,1);
% for i = 1:size(direct_step,1)
%     plotarm(h,direct_step(i,:));
% end
% grid on;
% axis equal
% %legend('x axis','y axis','z axis')
% xlabel('y [m]')
% ylabel('z [m]')
% zlabel('x [m]')
% 
% %# collect axes handles
% axH = findall(gcf,'type','axes');
% 
% %# set the y-limits of all axes (see axes properties for 
% %# more customization possibilities)
% set(axH,'xlim',[-0.35 0.05])
% set(axH,'ylim',[-0.15 0.35])
% 
% h = figure(2);
% subplot(1,2,1);
% for i = 1:size(gr_vel,1)
%     plotarm(h,gr_vel(i,:));
% end
% grid on;
% axis equal
% %legend('x axis','y axis','z axis')
% xlabel('y [m]')
% ylabel('z [m]')
% zlabel('x [m]')
% 
% h = figure(2);
% subplot(1,2,2);
% for i = 1:size(gr_vel_fb,1)
%     plotarm(h,gr_vel_fb(i,:));
% end
% grid on;
% axis equal
% %legend('x axis','y axis','z axis')
% xlabel('y [m]')
% ylabel('z [m]')
% zlabel('x [m]')
% 
% %# collect axes handles
% axH = findall(gcf,'type','axes');
% 
% %# set the y-limits of all axes (see axes properties for 
% %# more customization possibilities)
% set(axH,'xlim',[-0.35 0.05])
% set(axH,'ylim',[-0.15 0.35])

h = figure(3);
for i = 1:size(tor,1)
    plotarm(h,tor(i,:));
end
grid on;
axis equal
%legend('x axis','y axis','z axis')
xlabel('y [m]')
ylabel('z [m]')
zlabel('x [m]')


%# collect axes handles
axH = findall(gcf,'type','axes');

%# set the y-limits of all axes (see axes properties for 
%# more customization possibilities)
set(axH,'xlim',[-0.35 0.05])
set(axH,'ylim',[-0.15 0.35])