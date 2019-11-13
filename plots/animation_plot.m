function animation_plot(flight,wp,targ,view_point)

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
%     xlim(wp.x_lim);
%     ylim(wp.y_lim);
    xlim([-2.5 2.5]);
    ylim([-1.0 1.0]);
    zlim(wp.z_lim);
    grid on
    hold on

    % Plot the target
    h_targ = plot3(targ.pos(1,1),targ.pos(2,1),targ.pos(3,1),'d','MarkerSize',8,'MarkerFaceColor','r');
    
    % Plot the Waypoints
    for k = 1:size(wp.x,2)
        [x_arrow, y_arrow, z_arrow] = frame_builder(wp.x(:,k));
        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
        
        h_wp = plot3(x,y,z,'linewidth',2);
    
        % Set the Correct Colors
        h_wp(1).Color = [1 0 0];
        h_wp(2).Color = [0 1 0];
        h_wp(3).Color = [0 0 1];
    end

    % Set Camera Angle
    daspect([1 1 1])
    
    switch view_point
        case 'persp'
            view(320,20);
%              zoom(1.8)
        case 'back'
            view(-90,0);
        case 'side'
            view(0,0);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the full trajectory
    plot3(x_act(1,:),x_act(2,:),x_act(3,:),'--k','linewidth',1);
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Initial Frame  
    [x_arrow, y_arrow, z_arrow] = frame_builder(x_act(:,1));
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

    h_persp = plot3(x,y,z,'linewidth',3);
    
    % Set the Correct Colors
    h_persp(1).Color = [1 0 0];
    h_persp(2).Color = [0 1 0];
    h_persp(3).Color = [0 0 1];

    % Labels and Legend
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Remainder with REAL-TIME
    curr_time = dt;
    while (curr_time <= t_act(end))
        tic
        k = ceil(curr_time/dt);
        
        [x_arrow, y_arrow, z_arrow] = frame_builder(x_act(:,k));
        h_persp = reassign(h_persp,x_arrow,y_arrow,z_arrow);
        
        if t_act(k) > targ.t_capture
            h_targ.XData = x_act(1,k);
            h_targ.YData = x_act(2,k);
            h_targ.ZData = x_act(3,k)-0.1;
        end
        
        drawnow
        
        curr_time = curr_time + toc;
    end
end
