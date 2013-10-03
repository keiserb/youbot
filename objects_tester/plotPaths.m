clear all
close all

filename = 'objects_new_1.bag'
topics = {'/objects','/optitrack'}

[objects, time_objects, optitrack, time_optitrack] = get_data_from_bags(filename,topics);

%transform opti-youbot into opti-object
yaw=optitrack(:,3);
yaw=yaw-yaw(1);
for i=1:length(yaw)
x(i)=optitrack(i,1)*cos(-yaw(i))-optitrack(i,2)*sin(-yaw(i));
y(i)=optitrack(i,1)*sin(-yaw(i))+optitrack(i,2)*cos(-yaw(i));
end
x=x';
y=y';
optitrack=[x,y,yaw];

objects_fit=objects(1:size(objects,1),:);
optitrack_fit=optitrack(1:size(objects,1),:);

% % least squares
% obj_x=kron(eye(size(objects_fit,1))*objects_fit(:,1),eye(2));
% obj_y=kron(eye(size(objects_fit,1))*objects_fit(:,2),[0,-1;1,0]);
% 
% opti_x=kron(eye(size(objects_fit,1))*optitrack_fit(:,1),[1;0]);
% opti_y=kron(eye(size(objects_fit,1))*optitrack_fit(:,2),[0;1]);
% 
% theta=kron(ones(size(objects_fit,1),1),eye(2));
% 
% A=[obj_x+obj_y,theta];
% 
% b=opti_x+opti_y;
% 
% x=A\b

my_error_function = @(x)error_function(x,objects_fit,optitrack_fit);
[offset,error]=fminsearch(my_error_function,[0,0,0]);

t_obj=transform_point(offset,optitrack);
figure(1)
hold on
plot(objects(:,1),objects(:,2),'Color','b','Marker','x');
plot(optitrack(:,1),optitrack(:,2),'Color','g','Marker','x');
plot(t_obj(:,1),t_obj(:,2),'Color','k');
% grid on
% axis equal
% for i=1:size(objects,1)
%     plot([t_obj(i,1);optitrack_fit(i,1)],[t_obj(i,2);optitrack_fit(i,2)],'Color','r','Marker','x');
% end