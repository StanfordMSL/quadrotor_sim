function debug_plot(time,states,quat,speed)

    vect_x = [1 0 0]';
    vect_y = [0 1 0]';
    vect_z = [0 0 1]';

    % Construct Rotation Matrices
    bRw = quat2rotm(quat(:,1)');
    pos = states(1:3,1);
    
    x_arrow = [pos pos+(bRw*vect_x)];
    y_arrow = [pos pos+(bRw*vect_y)];
    z_arrow = [pos pos+(bRw*vect_z)];

    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';
    
    subplot(2,2,1)
    h_top = plot3(x,y,z,'linewidth',5);
    view(90,90)
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
        
    subplot(2,2,2)
    h_side = plot3(x,y,z,'linewidth',5);
    view(90,0)
    h_side(1).Color = [1 0 0];
    h_side(2).Color = [0 1 0];
    h_side(3).Color = [0 0 1];
    legend('x','y','z');
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    grid on
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([-5 5]);
    title('Side');
    pbaspect([1 1 1])

    subplot(2,2,3)
    h_back = plot3(x,y,z,'linewidth',5);
    view(0,0)
    h_back(1).Color = [1 0 0];
    h_back(2).Color = [0 1 0];
    h_back(3).Color = [0 0 1];
    legend('x','y','z');
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    grid on
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([-5 5]);
    title('Back');
    pbaspect([1 1 1])
    
    subplot(2,2,4)
    h_persp = plot3(x,y,z,'linewidth',5);
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
    title('Perspective');
    pbaspect([1 1 1])
     
    curr_time = time.act_dt;
    while (curr_time <= time.t_act(end))
        tic
        k = ceil(curr_time/time.act_dt);
        
        bRw = quat2rotm(quat(:,k)');
    
%         pos = states(1:3,k);
        pos(3,1) = states(3,k);

        x_arrow = [pos pos+(bRw*vect_x)];
        y_arrow = [pos pos+(bRw*vect_y)];
        z_arrow = [pos pos+(bRw*vect_z)];

        h_top = reassign(h_top,x_arrow,y_arrow,z_arrow);
        h_side = reassign(h_side,x_arrow,y_arrow,z_arrow);
        h_back = reassign(h_back,x_arrow,y_arrow,z_arrow);
        h_persp = reassign(h_persp,x_arrow,y_arrow,z_arrow);
        
        drawnow
        
%         text_out = strcat('Time: ',num2str(k*dt));
%         disp(text_out);
%         pause;
        curr_time = curr_time + toc;
    end
end
