function fig_h = animation_plot(flight, varargin)
    t_act = flight.t_act;
    x_act = flight.x_act;
    
    dt = t_act(1,2)-t_act(1,1);
    
    fig_h = figure(2);
    clf
    if(~isempty(varargin))
        hold on;
        plot_camera(varargin{1}, gcf);
    end
%     subplot(2,2,3);
%     plot(time.t_act,states(3,:));
%     title('Z Position (over time)');
%     
%     subplot(2,2,4);
%     plot(states(1,:),states(2,:));
%     title('XY Position (net)');
%     xlim([-5 5]);
%     ylim([-5 5]);

    % Plot Craft Angles
    vect_x = [0.2 0.0 0.0]';
    vect_y = [0.0 0.2 0.0]';
    vect_z = [0.0 0.0 0.2]';


    % Construct Rotation Matrices
    q0 = sqrt(1-x_act(7:9,1)'*x_act(7:9,1));
    quat = [q0 ; x_act(7:9,1)];
    
    bRw = quat2rotm(quat');
    pos = x_act(1:3,1);
    
    x_arrow = [pos pos+(bRw*vect_x)];
    y_arrow = [pos pos+(bRw*vect_y)];
    z_arrow = [pos pos+(bRw*vect_z)];

    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
    
%     subplot(2,2,[1 2]);
    h_persp = plot3(x,y,z,'linewidth',3);
    hold on
    plot3(x_act(1,:),x_act(2,:),x_act(3,:));
    h_persp(1).Color = [1 0 0];
    h_persp(2).Color = [0 1 0];
    h_persp(3).Color = [0 0 1];
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    legend('X','Y','Z','trajectory');
    grid on
    xlim([-8.1 8.1]);
    ylim([-3.2 3.2]);
    zlim([0 3]);
    daspect([1 1 1])
    view(320,20);
    
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
