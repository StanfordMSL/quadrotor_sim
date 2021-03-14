function [x_arrow, y_arrow, z_arrow] = frame_builder(x_data)

    % Body Frame Axes
    vect_x = [0.0885 0.0000 0.0000]';
    vect_y = [0.0000 0.0885 0.0000]';
    vect_z = [0.0000 0.0000 0.0200]';
    
    % Construct Rotation Matrix
    if size(x_data,1) == 13
        quat = x_data(7:10,1);
        bRw = quat2rotm(quat');
    else
        angles = [x_data(4,1) 0 0];
        bRw = eul2rotm(angles);
    end
    % Determine World Frame Pose of Craft Axes
    pos = x_data(1:3,1);
    
    x_arrow = [pos-(bRw*vect_x) pos+(bRw*vect_x)];
    y_arrow = [pos-(bRw*vect_y) pos+(bRw*vect_y)];
    z_arrow = [pos pos+(bRw*vect_z)];
end