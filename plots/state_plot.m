function state_plot(flight)

    t_act = flight.t_act;
    dt = t_act(1,2)-t_act(1,1);
    
    quat = zeros(4,length(flight.t_act));
    for k = 1:length(flight.t_act)
        q0 = sqrt(1-flight.x_act(7:9,k)'*flight.x_act(7:9,k));
        quat(:,k) = [q0 ; flight.x_act(7:9,k)];
    end
    pos = flight.x_act(1:3,:);
    vel = flight.x_act(4:6,:);
    omega = flight.x_act(10:12,:);
    
    angle = zeros(3,size(quat,2));
    for k = 2:size(quat,2)
        angle(:,k) = angle(:,k-1)+dt.*omega(:,k-1);
    end
    
    figure(1)
    % Plot Craft Angles
%     subplot(4,1,1)
%     plot(t_act,quat(1,:));
%     title('q0');
% 
%     subplot(4,1,2)
%     plot(t_act,quat(2,:));
%     title('q1');
% 
%     subplot(4,1,3)
%     plot(t_act,quat(3,:));
%     title('q2');
% 
%     subplot(4,1,4)
%     plot(t_act,quat(4,:));
%     title('q3');

    subplot(4,2,2)
    plot(t_act,angle);
    legend('\theta','\phi','\psi')
    title('Craft Angle (rad)');
    
    subplot(4,2,4)
    plot(t_act,omega);
    legend('\omega_x','\omega_y','\omega_z')
    title('Craft Angle Rates (rad/s)');

    subplot(4,2,6)
    plot(t_act,pos);
    legend('X','Y','Z')
    title('World Frame Position (m)');
    
    subplot(4,2,8)
    plot(t_act,vel);
    legend('v_x','v_y','v_z')
    title('World Frame Velocities (m/s)');

    vect_x = [2 0 0]';
    vect_y = [0 2 0]';
    vect_z = [0 0 2]';

    % Construct Rotation Matrices
    bRw = quat2rotm(quat(:,1)');
    anchor = pos(1:3,1);
    
    x_arrow = [anchor anchor+(bRw*vect_x)];
    y_arrow = [anchor anchor+(bRw*vect_y)];
    z_arrow = [anchor anchor+(bRw*vect_z)];

    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
    
    subplot(4,2,[1 3]);
    h_persp = plot3(x,y,z,'linewidth',3);
    h_persp(1).Color = [1 0 0];
    h_persp(2).Color = [0 1 0];
    h_persp(3).Color = [0 0 1];
    legend('x','y','z');
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    grid on
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([-5 5]);
    title('Attitude');
    pbaspect([1 1 1])
    
    subplot(4,2,[5 7]);
    h_top = plot3(x,y,z,'linewidth',5);
    view(0,90)
    h_top(1).Color = [1 0 0];
    h_top(2).Color = [0 1 0];
    h_top(3).Color = [0 0 1];
    legend('x','y','z');
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    grid on
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([-5 5]);
    title('Top');
    pbaspect([1 1 1])
    
%     curr_time = dt;
%     while (curr_time <= t_act(end))
%         tic
%         k = ceil(curr_time/dt);
%         
%         bRw = quat2rotm(quat(:,k)');
%     
%         x_arrow = [anchor anchor+(bRw*vect_x)];
%         y_arrow = [anchor anchor+(bRw*vect_y)];
%         z_arrow = [anchor anchor+(bRw*vect_z)];
% 
%         h_persp = reassign(h_persp,x_arrow,y_arrow,z_arrow);
%         h_top   = reassign(h_top,x_arrow,y_arrow,z_arrow);
% 
%         drawnow
%         
%         curr_time = curr_time + toc;
%     end
end
