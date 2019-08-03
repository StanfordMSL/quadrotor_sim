function presentation_plot(time,states,quat,mu_ekf,mu_ukf) 
    figure(1)
    subplot(2,1,1)
    rodriguez_plot(time,states,quat,mu_ekf,mu_ukf);
    
    subplot(2,1,2)
    distance_plot(time,states,quat,mu_ekf,mu_ukf);

    figure(2)
    clf
    % Plot Craft Angles
    vect_x = [0.3 0.0 0.0]';
    vect_y = [0.0 0.3 0.0]';
    vect_z = [0.0 0.0 0.3]';

    % Construct Rotation Matrices
    bRw = quat2rotm(quat(:,1)');
    pos = states(1:3,1);
    
    x_arrow = [pos pos+(bRw*vect_x)];
    y_arrow = [pos pos+(bRw*vect_y)];
    z_arrow = [pos pos+(bRw*vect_z)];

    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
    
%     subplot(2,2,[1 2]);
    h_persp = plot3(x,y,z,'linewidth',2);
    h_persp(1).Color = [1 0 0];
    h_persp(2).Color = [0 1 0];
    h_persp(3).Color = [0 0 1];
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    title('3D Plot');
    grid on
    xlim([-2 2]);
    ylim([-3 3]);
    zlim([0 2]);
    daspect([1 1 1])
    view(75,10);
    
    curr_time = time.act_dt;
    while (curr_time <= time.t_act(end))
        tic
        k = ceil(curr_time/time.act_dt);
        
        bRw = quat2rotm(quat(:,k)');
    
        pos = states(1:3,k);

        x_arrow = [pos pos+(bRw*vect_x)];
        y_arrow = [pos pos+(bRw*vect_y)];
        z_arrow = [pos pos+(bRw*vect_z)];

        h_persp = reassign(h_persp,x_arrow,y_arrow,z_arrow);

        drawnow
        
        curr_time = curr_time + toc;
    end
end
