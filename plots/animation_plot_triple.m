function animation_plot_triple(f1,f2,f3,wp,targ,view_point,wp_show)

    map = wp.map;
    
    t_act_1 = f1.t_act;
    x_act_1 = f1.x_act;
    x_act_2 = f2.x_act;
    x_act_3 = f3.x_act;
    
    dt = t_act_1(1,2)-t_act_1(1,1);

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
%     zlim(wp.z_lim);
    xlim([-4.0 4.0]);
%     ylim([-1.0 1.0]);
    zlim([ 0.0 2.0]);
    grid on
    hold on
    set(gcf,'color','white')

    % Plot the target
    h_targ = plot3(targ.pos(1,1),targ.pos(2,1),targ.pos(3,1),'d','MarkerSize',8,'MarkerFaceColor','r');
    
    switch wp_show
        case 'show'
            % Plot the Waypoints
            for k = 1:size(wp.x,2)
                [x_arrow_1, y_arrow_1, z_arrow_1] = frame_builder(wp.x(:,k));
                x1 = [x_arrow_1(1,:) ; y_arrow_1(1,:) ; z_arrow_1(1,:)]';
                y1 = [x_arrow_1(2,:) ; y_arrow_1(2,:) ; z_arrow_1(2,:)]';
                z1 = [x_arrow_1(3,:) ; y_arrow_1(3,:) ; z_arrow_1(3,:)]';

                h_wp = plot3(x1,y1,z1,'linewidth',2);

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
%              zoom(1.8)
        case 'back'
            view(-90,0);
        case 'side'
            view(0,0);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the full trajectory
    plot3(x_act_1(1,:),x_act_1(2,:),x_act_1(3,:),'--r','linewidth',1);
    plot3(x_act_2(1,:),x_act_2(2,:),x_act_2(3,:),'--g','linewidth',1);
    plot3(x_act_3(1,:),x_act_3(2,:),x_act_3(3,:),'--b','linewidth',1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Initial Frame
  
    % Drone #1
    [x_arrow_1, y_arrow_1, z_arrow_1] = frame_builder(x_act_1(:,1));
    x1 = [x_arrow_1(1,:) ; y_arrow_1(1,:) ; z_arrow_1(1,:)]';
    y1 = [x_arrow_1(2,:) ; y_arrow_1(2,:) ; z_arrow_1(2,:)]';
    z1 = [x_arrow_1(3,:) ; y_arrow_1(3,:) ; z_arrow_1(3,:)]';

    h_persp_1 = plot3(x1,y1,z1,'linewidth',3);
    
    % Set the Correct Colors
    h_persp_1(1).Color = [1 0 0];
    h_persp_1(2).Color = [1 0 0];
    h_persp_1(3).Color = [1 0 0];

    % Drone #2
    [x_arrow_2, y_arrow_2, z_arrow_2] = frame_builder(x_act_2(:,1));
    x2 = [x_arrow_2(1,:) ; y_arrow_2(1,:) ; z_arrow_2(1,:)]';
    y2 = [x_arrow_2(2,:) ; y_arrow_2(2,:) ; z_arrow_2(2,:)]';
    z2 = [x_arrow_2(3,:) ; y_arrow_2(3,:) ; z_arrow_2(3,:)]';

    h_persp_2 = plot3(x2,y2,z2,'linewidth',3);
    
    % Set the Correct Colors
    h_persp_2(1).Color = [0 1 0];
    h_persp_2(2).Color = [0 1 0];
    h_persp_2(3).Color = [0 1 0];
    
    % Drone #3
    [x_arrow_3, y_arrow_3, z_arrow_3] = frame_builder(x_act_3(:,1));
    x3 = [x_arrow_3(1,:) ; y_arrow_3(1,:) ; z_arrow_3(1,:)]';
    y3 = [x_arrow_3(2,:) ; y_arrow_3(2,:) ; z_arrow_3(2,:)]';
    z3 = [x_arrow_3(3,:) ; y_arrow_3(3,:) ; z_arrow_3(3,:)]';

    h_persp_3 = plot3(x3,y3,z3,'linewidth',3);
    
    % Set the Correct Colors
    h_persp_3(1).Color = [0 0 1];
    h_persp_3(2).Color = [0 0 1];
    h_persp_3(3).Color = [0 0 1];
    
    % Labels and Legend
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');

    legend([h_persp_1(1) h_persp_2(1) h_persp_3(1)],'iLEQR_{oa}, \gamma = -3e^{-6}','iLEQR_{oa}, \gamma = 1e^{-9}','iLEQR_{oa}, \gamma = 3e^{-6}',...
            'Location','best');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Remainder with REAL-TIME
    curr_time = dt;
    while (curr_time <= t_act_1(end))
        tic
        k = ceil(curr_time/dt);
        
        % Drone #1
        [x_arrow_1, y_arrow_1, z_arrow_1] = frame_builder(x_act_1(:,k));
        h_persp_1 = reassign(h_persp_1,x_arrow_1,y_arrow_1,z_arrow_1);
                
        if t_act_1(k) > f1.t_capture
            h_targ.XData = x_act_1(1,k);
            h_targ.YData = x_act_1(2,k);
            h_targ.ZData = x_act_1(3,k)-0.1;
        end
        
        % Drone #2
        [x_arrow_2, y_arrow_2, z_arrow_2] = frame_builder(x_act_2(:,k));
        h_persp_2 = reassign(h_persp_2,x_arrow_2,y_arrow_2,z_arrow_2);

        % Drone #3
        [x_arrow_3, y_arrow_3, z_arrow_3] = frame_builder(x_act_3(:,k));
        h_persp_3 = reassign(h_persp_3,x_arrow_3,y_arrow_3,z_arrow_3);
        
        drawnow
        
        curr_time = curr_time + toc;
    end
end