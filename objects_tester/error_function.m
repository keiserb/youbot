function [ error ] = error_function(offset, objects, optitrack)

transformed_objects=transform_point(offset,optitrack)
x_error=transformed_objects(:,1)-objects(:,1);
y_error=transformed_objects(:,2)-objects(:,2);

error=norm((x_error.^2+y_error.^2).^(1/2),2)


end

