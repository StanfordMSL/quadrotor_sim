function direct_plot(x_act,wp,view_point,wp_show)

    map = wp.map;
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
    figure(1)
%     clf
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generate flight room map
    gate_h = plot3(map(1,:)',map(2,:)',map(3,:)','b');
    gate_h.LineWidth = 5;
    xlim(wp.x_lim);
    ylim(wp.y_lim);
    zlim(wp.z_lim);
%     xlim([-2.5 2.5]);
%     ylim([-1.0 1.0]);
%     zlim([ 0.0 2.0]);

    grid on
    hold on
    set(gcf,'color','white')
   
    switch wp_show
        case 'show'
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
        case 'hide'
    end

    % Set Camera Angle
    daspect([1 1 1])
    
    switch view_point
        case 'persp'
            view(320,20);
        case 'debug'
            view(280,5);
            zoom(1.5)
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
end
