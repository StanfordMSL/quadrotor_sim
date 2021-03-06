function fout_animation_plot(f_out,map,obj,view_point)
    % Unpack som e stuff
    n_dr = size(f_out,3);
    n_fr = size(f_out,2);
    
    % Reset window
    figure(2)
    clf
    grid on
    hold on
    set(gcf,'color','white')
    daspect([1 1 1])
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    
    % Define Map Limits
    xlim(map.x_lim);
    ylim(map.y_lim);
    zlim(map.z_lim);

    % Generate waypoints
    for k = 1:size(obj.x,2)
        [x_arrow, y_arrow, z_arrow] = frame_builder(obj.x(:,k));
        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

        h_wp = plot3(x,y,z,'linewidth',2);

        % Set the Correct Colors
        h_wp(1).Color = [1 0 1];
        h_wp(2).Color = [1 0 1];
        h_wp(3).Color = [1 0 1];
    end

    % Set Camera Angle
    switch view_point
        case 'persp'
            view(320,20);
            zoom(2)
        case 'back'
            view(-90,0);
        case 'side'
            view(0,0);
    end
    
    % Plot the full trajectory
    plot3(f_out(1,:,1),f_out(2,:,1),f_out(3,:,1),'--k','linewidth',1);

        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for k_dr = n_dr:-1:1
        drone(k_dr).id =  text(f_out(1,1,k_dr),f_out(2,1,k_dr),f_out(3,1,k_dr),num2str(k_dr));
    end
    
    % Plot the Initial Frame  
    for k_dr = 1:n_dr
        [x_arrow, y_arrow, z_arrow] = frame_builder(f_out(:,1,k_dr));
        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

        drone(k_dr).h_persp = plot3(x,y,z,'linewidth',3);

        % Set the Correct Colors
        drone(k_dr).h_persp(1).Color = [1 0 0];
        drone(k_dr).h_persp(2).Color = [0 1 0];
        drone(k_dr).h_persp(3).Color = [0 0 1];
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Animate the trajectory
    disp('[fast_animation]: plot is via frame steps and NOT live');

    for k_fr=1:n_fr
        for k_dr = 1:n_dr
            [x_arrow, y_arrow, z_arrow] = frame_builder(f_out(:,k_fr,k_dr));
            drone(k_dr).h_persp = reassign(drone(k_dr).h_persp,x_arrow,y_arrow,z_arrow);
            
            delete(drone(k_dr).id);
            drone(k_dr).id =  text(f_out(1,k_fr,k_dr),f_out(2,k_fr,k_dr),f_out(3,k_fr,k_dr),num2str(k_dr));
        end

        drawnow
        pause(0.1);
    end
end
