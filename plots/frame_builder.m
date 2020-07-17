function [x_arrow, y_arrow, z_arrow] = frame_builder(x_data)

    % Body Frame Axes
    vect_x = [0.08 0.00 0.00]';
    vect_y = [0.00 0.08 0.00]';
    vect_z = [0.00 0.00 0.02]';
    
    % Construct Rotation Matrix
    quat = x_data(7:10,1);
    bRw = quat2rotm(quat');
    
    % Determine World Frame Pose of Craft Axes
    pos = x_data(1:3,1);
    
    x_arrow = [pos-(bRw*vect_x) pos+(bRw*vect_x)];
    y_arrow = [pos-(bRw*vect_y) pos+(bRw*vect_y)];
    z_arrow = [pos pos+(bRw*vect_z)];
end