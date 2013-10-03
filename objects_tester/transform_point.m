function [ transformed_objects ] = transform_point( offset, objects )
%TRANSFORM_POINT Summary of this function goes here
%   Detailed explanation goes here
    x=offset(1)*ones(size(objects,1),1)+(objects(:,1)*cos(offset(3))-objects(:,2)*sin(offset(3)));
    y=offset(2)*ones(size(objects,1),1)+(objects(:,1)*sin(offset(3))+objects(:,2)*cos(offset(3)));
    transformed_objects=[x,y];
end

