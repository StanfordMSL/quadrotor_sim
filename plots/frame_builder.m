function [x_arrow, y_arrow, z_arrow] = frame_builder(x_data)

    % Body Frame Axes
    vect_x = [0.2 0.0 0.0]';
    vect_y = [0.0 0.2 0.0]';
    vect_z = [0.0 0.0 0.1]';
    
    % Construct Rotation Matrix
    quat = x_data(7:10,1);
    bRw = quat2rotm(quat');
    
    % Determine World Frame Pose of Craft Axes
    pos = x_data(1:3,1);
    
    x_arrow = [pos pos+(bRw*vect_x)];
    y_arrow = [pos pos+(bRw*vect_y)];
    z_arrow = [pos pos+(bRw*vect_z)];
end