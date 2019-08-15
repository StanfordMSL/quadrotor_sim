function wp = wp_init(traj,t_now,tf,plot_show)

base_gate = [ 0.00  0.00  0.00  0.00  0.00;...
             -0.30 -0.30  0.30  0.30 -0.30;...
             -0.15  0.15  0.15 -0.15 -0.15];
map = [0 0 0]';

switch traj
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'hover'
        N_wp = 2;      
        t = [0 tf];
        
        x = zeros(12,2);
        x(:,1) = [0 ; 0 ; 1 ; zeros(9,1)];
        x(:,2) = [0 ; 0 ; 1 ; zeros(9,1)];
                
        Q_key = [4 4]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'climb'
        N_wp = 2;
        t = [0 0.5 tf];
        
        x = zeros(12,2);
        x(:,2) = [0 ; 0 ; 1 ; zeros(3,1) ; 0 ; 0 ; 0.7 ; 0 ; 0 ; 0];
        x(:,3) = [0 ; 0 ; 1 ; zeros(3,1) ; 0 ; 0 ; 0.7 ; 0 ; 0 ; 0];
                
        Q_key = [4 4;...
                 4 4];
             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'line'
        N_wp = 3;
        t = [0 (1/3)*tf (2/3)*tf tf];
        
        x = zeros(12,3);
        x(:,2) = [0 ; 0 ; 1; zeros(9,1)];
        x(:,3) = [2 ; 0 ; 1; zeros(9,1)];
        x(:,4) = [2 ; 0 ; 1; zeros(9,1)];
        
        Q_key = [4 4;...
                 4 4;...
                 4 4];
             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'half-square'
        N_wp = 5;
        t = [0 (1/4)*tf (2/4)*tf (3/4)*tf tf];
        
        x = zeros(12,4);
        x(:,2) = [0 ; 0 ; 1; zeros(3,1) ; 0 ; 0 ;  0.7 ; 0 ; 0 ; 0];
        x(:,3) = [0 ; 2 ; 1; zeros(9,1)];
        x(:,4) = [2 ; 2 ; 1; zeros(3,1) ; 0 ; 0 ; -0.7 ; 0 ; 0 ; 0];
        x(:,5) = [2 ; 2 ; 1; zeros(9,1)];
        
        Q_key = [4 4;...
                 4 4;...
                 4 4;...
                 4 4];
             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'almost-square'
        N_wp = 6;
        t = [0 (1/5)*tf (2/5)*tf (3/5)*tf (4/5)*tf tf];
        
        x = zeros(12,5);
        x(:,2) = [0 ; 0 ; 1; zeros(9,1)];
        x(:,3) = [0 ; 2 ; 1; zeros(9,1)];
        x(:,4) = [2 ; 2 ; 1; zeros(9,1)];
        x(:,5) = [2 ; 0 ; 1; zeros(9,1)];
        x(:,6) = [2 ; 0 ; 1; zeros(9,1)];
        
        Q_key = [4 4;...
                 4 4;...
                 4 4;...
                 4 4;...
                 4 4];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'square'
        N_wp = 6;
        t = [0 (1/6)*tf (2/6)*tf (3/6)*tf (4/6)*tf (5/6)*tf tf]';
        
        x = zeros(12,7);
        x(:,2) = [ 0 ; 0 ; 1; zeros(9,1)];
        x(:,3) = [ 0 ; 2 ; 1; zeros(9,1)];
        x(:,4) = [ 2 ; 2 ; 1; zeros(9,1)];
        x(:,5) = [ 2 ; 0 ; 1; zeros(9,1)];
        x(:,6) = [ 0 ; 0 ; 1; zeros(9,1)];
        x(:,7) = [ 0 ; 0 ; 0; zeros(9,1)];
                
        Q_key = [4 4;...
                 4 4;...
                 4 4;...
                 4 4;...
                 4 4;...
                 4 4];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    case 'slit'
        N_wp = 2;
        t = [0 (1/2)*tf tf];
        
        x = zeros(12,N_wp);
        x(:,1) = [-1.0 ; 0.0 ; 1.0 ; zeros(9,1)];
        x(:,2) = [ 0.0 ; 0.0 ; 1.0 ; zeros(3,1) ; 0.3 ; 0.0 ; 0.0 ; zeros(3,1)];    % Tilt (through the hole)
        x(:,3) = [ 1.0 ; 0.0 ; 1.0 ; zeros(9,1)];
                 
        Q_key = [3 3;...
                 4 4];
               
        q0 = sqrt(1-x(7:9,2)'*x(7:9,2));
        quat = [q0 ; x(7:9,2)];
        bRw = quat2rotm(quat');

        map = bRw*base_gate + [0 0 1]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    otherwise
        disp('[wp_init]: No trajectory loaded!');
end

wp.gate = base_gate;
wp.t = t_now + t;
wp.x = x;
wp.N_wp = N_wp;
wp.tf = tf;

wp.x_lim = [-8.1 8.1];
wp.y_lim = [-3.2 3.2];
wp.z_lim = [0 3];

wp.Q_key = Q_key;
wp.map = map;

switch plot_show
    case 'plot'
        figure(1)
        clf
        
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

        xlabel('x-axis');
        ylabel('y-axis');
        zlabel('z-axis');
        grid on
        xlim([-8.1 8.1]);
        ylim([-3.2 3.2]);
        zlim([0 3]);
        daspect([1 1 1])
        view(-50,10);
    case 'no plot'
        % Do Nothing
    otherwise
        % Do Nothing
end
