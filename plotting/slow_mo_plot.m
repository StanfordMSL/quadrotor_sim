function slow_mo_plot(flight,obj,targ,view_point,wp_show)
    
    t_act = flight.t_act;
    x_act = flight.x_act;
    
    dt = t_act(1,2)-t_act(1,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
%     figure(3)
%     clf
    figure(4)
    subplot(4,4,[2:4,6:8,10:12,14:16])
    cla
    set(gca,'ColorOrder','factory')
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generate flight room map
    pnts_gate_rdr = [obj.p_gc obj.p_gc(:,1)];  % render points need to terminate at start
    gate_h = plot3(pnts_gate_rdr(1,:)',pnts_gate_rdr(2,:)',pnts_gate_rdr(3,:)');
    gate_h.LineWidth = 3;
%     xlim(obj.x_lim);
%     ylim(obj.y_lim);
%     zlim(obj.z_lim);
    xlim([-2.1 2.1]);
    ylim([-0.2 1.0]);
    zlim([ 0.0 1.5]);

    grid on
    hold on
    set(gcf,'color','white')

    % Plot the target
    h_targ = plot3(targ.pos(1,1),targ.pos(2,1),targ.pos(3,1),'d','MarkerSize',8,'MarkerFaceColor','r');
    
    switch wp_show
        case 'show'
            % Plot the Waypoints
            for k = 1:size(obj.wp_arr,2)
                [x_arrow, y_arrow, z_arrow] = frame_builder(obj.wp_arr(:,k));
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
        case 'back'
            view(-90,0);
        case 'top'
            view(0,90);
            zoom(3)
        case 'side'
            view(0,0);
        case 'nice'
            view(-60,20);
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
    for k = 1:10:size(x_act,2)
        
        [x_arrow, y_arrow, z_arrow] = frame_builder(x_act(:,k));
        h_persp = reassign(h_persp,x_arrow,y_arrow,z_arrow);
        
        if t_act(k) > flight.t_capture
            h_targ.XData = x_act(1,k);
            h_targ.YData = x_act(2,k);
            h_targ.ZData = x_act(3,k)-0.1;
        end
        
        drawnow
        
        pause(0.1);
    end
end
