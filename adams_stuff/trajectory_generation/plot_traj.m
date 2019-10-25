function plot_traj(wp)
    map = wp.map;
    x = wp.x;
    N_wp = wp.N_wp;
    gate_h = plot3(map(1,:)',map(2,:)',map(3,:)');
    gate_h.LineWidth = 3;
    hold on

    % Body Frame Axes
    vect_x = [0.2 0.0 0.0]';
    vect_y = [0.0 0.2 0.0]';
    vect_z = [0.0 0.0 0.1]';
    for k = 1:N_wp
        q0 = sqrt(1-x(7:9,k)'*x(7:9,k));
        quat = [q0 ; x(7:9,k)];
        bRw = quat2rotm(quat');

        pos = x(1:3,k);

        x_arrow = [pos pos+(bRw*vect_x)];
        y_arrow = [pos pos+(bRw*vect_y)];
        z_arrow = [pos pos+(bRw*vect_z)];

        pos_x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        pos_y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        pos_z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

        quad_h = plot3(pos_x,pos_y,pos_z,'linewidth',2);
        quad_h(1).Color = [1 0 0];
        quad_h(2).Color = [0 1 0];
        quad_h(3).Color = [0 0 1];

        text(pos(1),pos(2),pos(3),num2str(k));
    end
    hold on
    plot3(x(1,:), x(2,:), x(3,:), 'b-')

    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    grid on
    xlim([-8.1 8.1]);
    ylim([-3.2 3.2]);
    zlim([0 3]);
    daspect([1 1 1])
    view(-50,10);
end