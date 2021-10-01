function [x_arrow, y_arrow] = frame_builder(x_data)

    vect_x = [0.10 ; 0.00];
    vect_y = [0.00 ; 0.10];
    % Construct Rotation Matrix
    theta = x_data(3,1);
    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];
    % Determine World Frame Pose of Craft Axes
    pos = x_data(1:2,1);
    
    x_arrow = [pos pos+(R*vect_x)];
    y_arrow = [pos pos+(R*vect_y)];
end