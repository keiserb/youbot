clear c abs_err direct_step mm_step mm_step_all gr_vel gr_vel_fb tor tor_all
close all
parameter = load_parameter();
joint_offsets = parameter.joint_offsets_deg/180*pi;

x=0.0;
y=-0.25;
t_step=0.02;
fid = fopen('grasp_direct_step.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    direct_step(i,1:5)=read(1:5,i);
    direct_step(i,1:5)=youbot_joints_2_matlab(direct_step(i,1:5));
    ds_pos(1:3,i)=compute_forward_kinematics(direct_step(i,1:5));
    pos_err_ds(1,i)=ds_pos(1,i)-x;
    pos_err_ds(2,i)=ds_pos(2,i)-y;
    abs_err_ds(i)=norm(pos_err_ds(:,i),2);
end
tds=t_step*size(direct_step,1)
fid = fopen('grasp_mm_step.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
k=1;
for i=1:n
    mm_step_all(i,1:5)=read(1:5,i);
    mm_step_all(i,1:5)=youbot_joints_2_matlab(mm_step_all(i,1:5));
    if(mod(i-1,3)==0)
        mm_step(k,1:5)=mm_step_all(i,1:5);
        mm_pos(1:3,k)=compute_forward_kinematics(mm_step(k,1:5));
        pos_err_mm(1,k)=mm_pos(1,k)-x;
        pos_err_mm(2,k)=mm_pos(2,k)-y;
        abs_err_mm(k)=norm(pos_err_mm(:,k),2);
        k=k+1;
    end
end
tmms=t_step*size(mm_step_all,1)
fid = fopen('grasp_vel.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    gr_vel(i,1:5)=read(1:5,i);
    gr_vel(i,1:5)=youbot_joints_2_matlab(gr_vel(i,1:5));
    v_pos(1:3,i)=compute_forward_kinematics(gr_vel(i,1:5));
    pos_err_v(1,i)=v_pos(1,i)-x;
    pos_err_v(2,i)=v_pos(2,i)-y;
    abs_err_v(i)=norm(pos_err_v(:,i),2);
end
tv=t_step*size(gr_vel,1)
fid = fopen('grasp_vel_fb.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
for i=1:n
    gr_vel_fb(i,1:5)=read(1:5,i);
    gr_vel_fb(i,1:5)=youbot_joints_2_matlab(gr_vel_fb(i,1:5));
    vfb_pos(1:3,i)=compute_forward_kinematics(gr_vel_fb(i,1:5));
    pos_err_vfb(1,i)=vfb_pos(1,i)-x;
    pos_err_vfb(2,i)=vfb_pos(2,i)-y;
    abs_err_vfb(i)=norm(pos_err_vfb(:,i),2);
end
tvfb=t_step*size(gr_vel_fb,1)
fid = fopen('tor_pos.txt','r')
read = fscanf(fid,'%f',[5,inf])
fclose(fid)
[m,n]=size(read)
k=1;
for i=1:n
    tor_all(i,1:5)=read(1:5,i);
    tor_all(i,1:5)=youbot_joints_2_matlab(tor_all(i,1:5));
    if(mod(i-1,5)==0)
        tor(k,1:5)=tor_all(i,1:5);
        tor_pos(1:3,k)=compute_forward_kinematics(tor(k,1:5));
        pos_err_tor(1,k)=tor_pos(1,k)-x;
        pos_err_tor(2,k)=tor_pos(2,k)-y;
        abs_err_tor(k)=norm(pos_err_tor(:,k),2);
        t_err_tor(k)=(i-1)*0.002;
        k=k+1;
    end
end
tt = 0.002*size(tor_all,1)

% h = figure(1);
% hold on;
% plot([-0.25,-0.25],[-0.02,-0.1],'Color',[0 0 0],'Marker','x')
% plot(ds_pos(2,:),ds_pos(3,:));
% grid on;
% axis equal
% %legend('x axis','y axis','z axis')
% xlabel('y [m]')
% ylabel('z [m]')
% zlabel('x [m]')
% 
% 
% %# collect axes handles
% axH = findall(gcf,'type','axes');
% 
% %# set the y-limits of all axes (see axes properties for 
% %# more customization possibilities)
% set(axH,'xlim',[-0.28 -0.22])
% set(axH,'ylim',[-0.12 0])
% 
% 
% h = figure(2);
% hold on;
% plot([-0.25,-0.25],[-0.02,-0.1],'Color',[0 0 0],'Marker','x')
% plot(mm_pos(2,:),mm_pos(3,:));
% grid on;
% axis equal
% %legend('x axis','y axis','z axis')
% xlabel('y [m]')
% ylabel('z [m]')
% zlabel('x [m]')
% 
% 
% %# collect axes handles
% axH = findall(gcf,'type','axes');
% 
% %# set the y-limits of all axes (see axes properties for 
% %# more customization possibilities)
% set(axH,'xlim',[-0.28 -0.22])
% set(axH,'ylim',[-0.12 0])
% 
% h = figure(3);
% hold on;
% plot([-0.25,-0.25],[-0.02,-0.1],'Color',[0 0 0],'Marker','x')
% plot(v_pos(2,:),v_pos(3,:));
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
% set(axH,'xlim',[-0.28 -0.22])
% set(axH,'ylim',[-0.12 0])
% 
% h = figure(4);
% hold on;
% plot(vfb_pos(2,:),vfb_pos(3,:));
% plot([-0.25,-0.25],[-0.02,-0.1],'Color',[0 0 0],'Marker','x')
% grid on;
% axis equal
% %legend('x axis','y axis','z axis')
% xlabel('y [m]')
% ylabel('z [m]')
% zlabel('x [m]')
% plot(ts,torA(:,3))
subplot(2,1,1)
plot(ts,torA(:,3),'b')
hold on
plot(ts,torV(:,3),'g')
plot(ts,torG(:,3),'r')
plot(ts,torA(:,3)+torV(:,1)+torG(:,3),'k')
xlabel('t [s]')
ylabel('Torque [Nm]')
grid on
legend('M*acc','C*vel','N','Total')
subplot(2,1,2)plot(ts,torA(:,3))
subplot(2,1,1)
plot(ts,torA(:,3),'b')
hold on
plot(ts,torV(:,3),'g')
plot(ts,torG(:,3),'r')
plot(ts,torA(:,3)+torV(:,1)+torG(:,3),'k')
xlabel('t [s]')
ylabel('Torque [Nm]')
grid on
legend('M*acc','C*vel','N','Total')
subplot(2,1,2)
plot(ts,vel(:,3),acc(:,3))
grid on
xlabel('t [s]')
ylabel('velocity [rad/s]')
plot(ts,vel(:,3),acc(:,3))
grid on
xlabel('t [s]')
ylabel('velocity [rad/s]')
% %# collect axes handles
% axH = findall(gcf,'type','axes');
% 
% %# set the y-limits of all axes (see axes properties for 
% %# more customization possibilities)
% set(axH,'xlim',[-0.28 -0.22])
% set(axH,'ylim',[-0.12 0])
% 
% h = figure(5);
% hold on;
% plot([-0.25,-0.25],[-0.02,-0.1],'Color',[0 0 0],'Marker','x')
% plot(tor_pos(2,:),tor_pos(3,:))
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
% set(axH,'xlim',[-0.28 -0.22])
% set(axH,'ylim',[-0.12 0])
% 
% h = figure(6);
% hold on;
% plot(t_err_tor,abs_err_tor);
% grid on;
% axis([0.0 1.7 0.0 0.003])
% xlabel('t [s]')
% ylabel('abs error [m]')

h = figure(7);
[m,n]=size(abs_err_tor);
abs_err(1:n,1)=abs_err_tor';
[m,n]=size(abs_err_mm);
abs_err(1:n,2)=abs_err_mm';
[m,n]=size(abs_err_vfb);
abs_err(1:n,3)=abs_err_vfb';
[m,n]=size(abs_err_v);
abs_err(1:n,4)=abs_err_v';
[m,n]=size(abs_err_ds);
abs_err(1:n,5)=abs_err_ds';
c=abs_err;
c(abs_err==0)=nan;
boxplot(c,'Label',{'tor' 'mm' 'vel fb' 'vel' 'dir'});
 xlabel('Method')
 ylabel('Position Error [m]')