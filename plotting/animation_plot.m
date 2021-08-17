function animation_plot(flight,obj,map,view_point,wp_show)
    
    t_act = flight.t_act;
    x_act = flight.x_act;
    x_ses = flight.x_ses;
    x_des = flight.x_des;
    
    dt = t_act(1,2)-t_act(1,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
%     figure(3)
%     clf
    figure(1)
    cla
    set(gca,'ColorOrder','factory')
    
    % Labels and Legend
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');

    % Define Map Limits
    xlim(map.x_lim);
    ylim(map.y_lim);
    zlim(map.z_lim);
    grid on
    hold on
    set(gcf,'color','white')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generate flight room map
    if any(ismember(fields(obj.gt),'p_ctr'))
        N_g = size(obj.gt.p_ctr,2);

        for k = 1:N_g
            % Gate(s) Present. Render.    
            p_G1 = obj.gt.p_box(:,1,k);
            p_G2 = obj.gt.p_box(:,2,k);
            p_G4 = obj.gt.p_box(:,4,k);

            r_12 = p_G2 - p_G1;
            r_14 = p_G4 - p_G1;
            n_G  = cross(r_14,r_12);
            gate_dir = obj.gt.p_box(:,1,k)+ (0.3.*n_G./norm(n_G));

            g_frame = [obj.gt.p_box(:,:,k) obj.gt.p_box(:,1,k) gate_dir];  % render points need to terminate at start

            gate_h = plot3(g_frame(1,:)',g_frame(2,:)',g_frame(3,:)','b');
            gate_h.LineWidth = 3;
            gate_h.Annotation.LegendInformation.IconDisplayStyle = 'off';
            hold on
        end
    else
        % No Gate. Carry On.
    end
    
    % Plot the target
    if strcmp(obj.type,'grasp') 
        h_targ = plot3(obj.pos(1,1),obj.pos(2,1),obj.pos(3,1),'d','MarkerSize',8,'MarkerFaceColor','r');
    	h_targ.Annotation.LegendInformation.IconDisplayStyle = 'off';
    else
        % Do Nothing
    end
    
    switch wp_show
        case 'show'
            % Plot the Waypoints
            for k = 1:size(obj.kf.x,2)
                [x_arrow, y_arrow, z_arrow] = frame_builder(obj.kf.x(:,k));
                x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
                y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
                z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

                h_wp = plot3(x,y,z,'linewidth',2);
                
                % Turn Off Legend
                h_wp(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
                h_wp(2).Annotation.LegendInformation.IconDisplayStyle = 'off';
                h_wp(3).Annotation.LegendInformation.IconDisplayStyle = 'off';

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
        case 'back'
            view(-90,0);
        case 'top'
        view(-90,90);
%             zoom(3)
        case 'side'
            view(0,0);
        case 'nice'
            view(-60,20);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the full trajectory
    plot3(x_act(1,:),x_act(2,:),x_act(3,:),'k','linewidth',1.5);
    plot3(x_des(1,:),x_des(2,:),x_des(3,:),'--b','linewidth',0.5);
    plot3(x_ses(1,:),x_ses(2,:),x_ses(3,:),'g','linewidth',0.5);

    legend('Actual','Desired','Estimator','Location','northwest');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Initial Frame  
    [x_arrow, y_arrow, z_arrow] = frame_builder(x_act(:,1));
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

    h_persp = plot3(x,y,z,'linewidth',3);
    
    % Turn Off Legend
    h_persp(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
    h_persp(2).Annotation.LegendInformation.IconDisplayStyle = 'off';
    h_persp(3).Annotation.LegendInformation.IconDisplayStyle = 'off';
                
    % Set the Correct Colors
    h_persp(1).Color = [1 0 0];
    h_persp(2).Color = [0 1 0];
    h_persp(3).Color = [0 0 1];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Remainder with REAL-TIME
    curr_time = dt;
    while (curr_time <= t_act(end))
        tic
        k = ceil(curr_time/dt);
        
        [x_arrow, y_arrow, z_arrow] = frame_builder(x_act(:,k));
        h_persp = reassign(h_persp,x_arrow,y_arrow,z_arrow);
        
        if t_act(k) > flight.t_capture
            h_targ.XData = x_act(1,k);
            h_targ.YData = x_act(2,k);
            h_targ.ZData = x_act(3,k)-0.1;
        end
        
        drawnow
        
        curr_time = curr_time + toc;
    end
end
