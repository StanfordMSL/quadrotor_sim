function animation_plot_and_bbs(flight, wp, traj_mat, bb_mat, t_mat, bb_rc_list_mat, camera, view_point)

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
    
    switch view_point
        case 'persp'
            view(320,20);
%             zoom(1.8)
        case 'back'
            view(-90,0);
    end
    
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
    
    
    figure(3433); axis equal; hold on;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Remainder with REAL-TIME
    k_est = 1;
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
        
        while k_est <= length(t_mat) && (t_mat(k_est) + 0.00001) < curr_time
            k_est = k_est + 1;
        end
        if k_est <= size(bb_mat, 2)
            figure(3433); clf;
            plot_bounding_angled_box(bb_mat(1:2, k_est), bb_mat(3, k_est), bb_mat(4, k_est), bb_mat(5, k_est), bb_rc_list_mat(:, :, k_est), traj_mat(1:3, k_est), traj_mat(7:10, k_est), camera)
        end
        drawnow
        curr_time = curr_time + toc;
    end
end
