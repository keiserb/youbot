function [objects, stamp_obj, opti, stamp_opti] = get_data_from_bags(filename, topics)
bag=importbag(filename,topics);

objects_x = bag.objects.objects0.pose.position.x;
objects_y = bag.objects.objects0.pose.position.y;

objects_q_x = bag.objects.objects0.pose.orientation.x;
objects_q_y = bag.objects.objects0.pose.orientation.y;
objects_q_z = bag.objects.objects0.pose.orientation.z;
objects_q_w = bag.objects.objects0.pose.orientation.w;

stamp_obj_temp = bag.objects.objects0.header.stamp;

x=c2m(objects_x);
x=x';
y=c2m(objects_y);
y=y';
q_x=c2m(objects_q_x);
q_y=c2m(objects_q_y);
q_z=c2m(objects_q_z);
q_w=c2m(objects_q_w);

t_obj_temp=c2m(stamp_obj_temp);

[yaw, pitch, roll] = quat2angle([q_w', q_x', q_y', q_z']);

objects=[x(~isnan(x)),y(~isnan(y)),yaw(~isnan(yaw))];
stamp_obj=t_obj_temp(~isnan(t_obj_temp))';
stamp_obj=stamp_obj-stamp_obj(1);

opti_x = bag.optitrack.pose.position.x;
opti_y = bag.optitrack.pose.position.y;

opti_q_x = bag.optitrack.pose.orientation.x;
opti_q_y = bag.optitrack.pose.orientation.y;
opti_q_z = bag.optitrack.pose.orientation.z;
opti_q_w = bag.optitrack.pose.orientation.w;

stamp_opti_temp=bag.optitrack.header.stamp;
stamp_opti_temp=stamp_opti_temp-stamp_opti_temp(1);

[yaw, pitch, roll] = quat2angle([opti_q_w,opti_q_x,opti_q_y,opti_q_z]);

opti_temp=[opti_x,opti_y,yaw];

for i=1:size(objects,1)
    [c idx]=min(abs(stamp_obj(i)-stamp_opti_temp));
    opti(i,1:3)=opti_temp(idx,:);
    stamp_opti(i)=stamp_opti_temp(idx);
end
stamp_opti=stamp_opti'
end
