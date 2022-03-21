function compare_plot(traj_nom,traj_act,obj,wp_show,step)
    
    T = traj_act.t_fmu;
    Xnom = traj_nom.X;
    Xact = traj_act.X;

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
    xlim(obj.map(1,:));
    ylim(obj.map(2,:));
    zlim(obj.map(3,:));
    grid on
    hold on
    set(gcf,'color','white')

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    switch wp_show
        case 'show'
            % Plot the Waypoints
            for k = 1:size(obj.x,2)
                [x_arrow, y_arrow, z_arrow] = frame_builder(obj.x(:,k));
                x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
                y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
                z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

                h_wp = plot3(x,y,z,'linewidth',2);
                
                % Turn Off Legend
                h_wp(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
                h_wp(2).Annotation.LegendInformation.IconDisplayStyle = 'off';
                h_wp(3).Annotation.LegendInformation.IconDisplayStyle = 'off';

                % Set the Correct Colors
                h_wp(1).Color = [0 0 0];
                h_wp(2).Color = [0 0 0];
                h_wp(3).Color = [0 0 0];
            end
        case 'hide'
    end

    % Set Camera Angle
    daspect([1 1 1])
    view(320,20);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the full trajectory
    plot3(Xnom(1,:),Xnom(2,:),Xnom(3,:),'g','linewidth',1.5);
    plot3(Xact(1,:),Xact(2,:),Xact(3,:),'r','linewidth',1.5);

    legend('Nominal','Actual','Location','northwest');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Stepped Sequence of Frames  

    for k = 1:step:size(Xnom,2)
        [x_arrow, y_arrow, z_arrow] = frame_builder(Xnom(:,k));

        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

        h_fr = plot3(x,y,z,'linewidth',2);

        % Turn Off Legend
        h_fr(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
        h_fr(2).Annotation.LegendInformation.IconDisplayStyle = 'off';
        h_fr(3).Annotation.LegendInformation.IconDisplayStyle = 'off';
        
        % Set the Correct Colors
        h_fr(1).Color = [1 0 0];
        h_fr(2).Color = [0 1 0];
        h_fr(3).Color = [0 0 1];
        hold on
    end

    for k = 1:step:size(Xact,2)
        [x_arrow, y_arrow, z_arrow] = frame_builder(Xact(:,k));

        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

        h_fr = plot3(x,y,z,'linewidth',2);

        % Turn Off Legend
        h_fr(1).Annotation.LegendInformation.IconDisplayStyle = 'off';
        h_fr(2).Annotation.LegendInformation.IconDisplayStyle = 'off';
        h_fr(3).Annotation.LegendInformation.IconDisplayStyle = 'off';
        
        % Set the Correct Colors
        h_fr(1).Color = [1 0 0];
        h_fr(2).Color = [0 1 0];
        h_fr(3).Color = [0 0 1];
        hold on
    end

end
