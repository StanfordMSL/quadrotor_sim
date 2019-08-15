function animation_plot(flight,wp)

    map = wp.map;
    
    t_act = flight.t_act;
    x_act = flight.x_act;
    
    dt = t_act(1,2)-t_act(1,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
    figure(2)
    clf
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generate flight room map
    gate_h = plot3(map(1,:)',map(2,:)',map(3,:)');
    gate_h.LineWidth = 3;
    xlim(wp.x_lim);
    ylim(wp.y_lim);
    zlim(wp.z_lim);
    grid on
    
    % Set Camera Angle
    daspect([1 1 1])
    view(320,20);
%     view(90,0);

    hold on
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Initial Frame
    
    % Body Frame Axes
    vect_x = [0.2 0.0 0.0]';
    vect_y = [0.0 0.2 0.0]';
    vect_z = [0.0 0.0 0.1]';
    
    % Construct Rotation Matrix
    q0 = sqrt(1-x_act(7:9,1)'*x_act(7:9,1));
    quat = [q0 ; x_act(7:9,1)];
    bRw = quat2rotm(quat');
    
    % Determine World Frame Pose of Craft Axes
    pos = x_act(1:3,1);
    
    x_arrow = [pos pos+(bRw*vect_x)];
    y_arrow = [pos pos+(bRw*vect_y)];
    z_arrow = [pos pos+(bRw*vect_z)];

    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
    
    % Plot It!
    h_persp = plot3(x,y,z,'linewidth',3);
    plot3(x_act(1,:),x_act(2,:),x_act(3,:));
    
    % Set the Correct Colors
    h_persp(1).Color = [1 0 0];
    h_persp(2).Color = [0 1 0];
    h_persp(3).Color = [0 0 1];

    % Labels and Legend
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    legend('X','Y','Z','trajectory');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Remainder with REAL-TIME
    curr_time = dt;
    while (curr_time <= t_act(end))
        tic
        k = ceil(curr_time/dt);
        
        q0 = sqrt(1-x_act(7:9,k)'*x_act(7:9,k));
        quat = [q0 ; x_act(7:9,k)];
    
        bRw = quat2rotm(quat');
    
        pos = x_act(1:3,k);

        x_arrow = [pos pos+(bRw*vect_x)];
        y_arrow = [pos pos+(bRw*vect_y)];
        z_arrow = [pos pos+(bRw*vect_z)];

        h_persp = reassign(h_persp,x_arrow,y_arrow,z_arrow);

        drawnow
        
        curr_time = curr_time + toc;
    end
end
